#include <SPI.h>
#include <WiFi.h>
#include <atomic>
#include <esp_task_wdt.h>  // 追加
#include "spi_manager.h"
#include "adc.h"
#include "led.h"
#include "wall_sensor.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "sensors.h"

// グローバルなクラスは、上から順に初期化される
ADC adc(get_shared_spi());
LED led;
WallSensor wall_sensor(adc);
IMU imu(get_shared_spi());
Motor motor;
Encoder encoder;
Sensors sensors(imu, wall_sensor, adc, encoder);

hw_timer_t* high_speed_timer = NULL;
std::atomic<uint32_t> timer_ticks(0);

QueueHandle_t cmd_queue; // コマンドキュー

struct SetMotorSpeedCommand {
    int16_t right_speed; // 右モーター速度
    int16_t left_speed;  // 左モーター速度
};

enum CommandID : uint8_t {
    CMD_SET_MOTOR_SPEED = 0x01,
    CMD_SET_SENSOR_ENABLE = 0x02,
    CMD_SET_SENSOR_DISABLE = 0x03,
    // 他のコマンドIDをここに追加
};

struct Command {
    uint8_t cmd_id;
    union {
        SetMotorSpeedCommand set_motor_speed;
        // 他のコマンド構造体をここに追加
    } parameter;
};

// ハイスピードタイマー割り込みハンドラ
// Core0RealtimeTaskで初期化されるため、CPU0で実行される
void IRAM_ATTR onHighSpeedTimer() {
    timer_ticks.fetch_add(1, std::memory_order_relaxed);
}

void Core0RealtimeTask(void* parameter) {
    // ハイスピードタイマー(1ms)の初期化
    high_speed_timer = timerBegin(1000); // 1000Hz = 1ms period
    timerAttachInterrupt(high_speed_timer, &onHighSpeedTimer);
    timerAlarm(high_speed_timer, 1, true, 0); // 1ms interval, auto-reload

    uint32_t last_tick = 0;

    while(1){
        while(timer_ticks.load(std::memory_order_relaxed) == last_tick) {
            // 待機
            asm volatile("nop");
        }
        led.green_on();
        last_tick = timer_ticks.load(std::memory_order_relaxed);

        // 1msごとの処理をここに記述
        // センサーデータの更新
        sensors.update();
        
        // コマンドキューの処理
        SetMotorSpeedCommand cmd;
        if (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
            motor.set_right_motor(cmd.right_speed);
            motor.set_left_motor(cmd.left_speed);
        }
        led.green_off();
    }
}

void setup() {
    Serial.begin(3000000);
    delay(100);
    
    // 1. ハードウェアWDT無効化
    disableCore0WDT();
    
    // WiFi/Bluetooth無効化
    WiFi.mode(WIFI_OFF);
    btStop();

    // 共有SPIとペリフェラルの初期化
    init_shared_spi();
    motor.begin();
    encoder.begin();
    imu.begin();

    // コマンドキューの作成
    cmd_queue = xQueueCreate(10, sizeof(SetMotorSpeedCommand));
    if (cmd_queue == NULL) {
        Serial.println("Failed to create command queue!");
        while(1);
    }

    // Core0でハイスピードタスクを開始
    BaseType_t result = xTaskCreatePinnedToCore(
        Core0RealtimeTask,
        "Core0RT",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // 最高優先度
        NULL,
        0  // Core 0
    );
    
    if (result != pdPASS) {
        Serial.println("Failed to create Core0 task!");
        while(1);
    }
    
    Serial.println("Core 0 realtime task started");
    Serial.println("System ready");
}
 
// Core1のループ関数
void loop() {
    // センサーデータの読み取り例
    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        
        float gyro = sensors.get_gyro_z();
        float vbatt = sensors.get_battery_voltage();
        uint16_t lf = sensors.get_lf();
        uint16_t ls = sensors.get_ls();
        uint16_t rs = sensors.get_rs();
        uint16_t rf = sensors.get_rf();
        uint16_t enc_r = sensors.get_right_wheel_angle();
        uint16_t enc_l = sensors.get_left_wheel_angle();
        
        Serial.printf("SEN,%.2f,%.2f,%u,%u,%u,%u,%u,%u\n",
                      gyro, vbatt, lf, ls, rs, rf, enc_r, enc_l);
    }
    
    // UARTコマンド受信例
    if (Serial.available()) {
        // e.g. "MOT,100,100\n"
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("MOT")) {
            // モーター速度設定: MOT,100,100
            int comma = cmd.indexOf(',');
            if (comma > 0) {
                SetMotorSpeedCommand motor_cmd;
                motor_cmd.right_speed = cmd.substring(1, comma).toInt();
                motor_cmd.left_speed = cmd.substring(comma + 1).toInt();
                
                if (xQueueSend(cmd_queue, &motor_cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("Motor: R=%d, L=%d\n", 
                                  motor_cmd.right_speed, motor_cmd.left_speed);
                } else {
                    Serial.println("Queue full!");
                }
            }
        }
    }
    
    delay(1);
}
