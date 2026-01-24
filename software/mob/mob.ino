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
#include "motion_controller.h"
#include <math.h>

// Target wheel speed [m/s] (updated from MOT command via cmd_queue)
static float target_vr_mps = 0.0f;
static float target_vl_mps = 0.0f;

// 前進コマンド（プロファイル走行）状態
static bool fwd_active = false;
static float fwd_v_cmd_mmps = 0.0f;     // 現在指令速度 [mm/s]
static float fwd_v_target_mmps = 0.0f;  // 目標巡航速度 [mm/s]
static float fwd_a_mmps2 = 0.0f;        // 加速度 [mm/s^2]（符号付き）
static float fwd_goal_dist_mm = 0.0f;   // 絶対目標距離 [mm]（Sensors::get_distance()基準）

// 停止コマンド（減速して停止する）状態
static bool stop_active = false;
static bool stop_done_pending = false;   // STOP目標距離到達後、停止確認してDONEを返す
static float stop_v_cmd_mmps = 0.0f;      // 現在指令速度 [mm/s]
static float stop_v_target_mmps = 0.0f;   // 目標速度（通常 0）[mm/s]
static float stop_a_mmps2 = 0.0f;         // 減速度 [mm/s^2]（負ではなく絶対値として扱う）
static float stop_goal_dist_mm = 0.0f;    // 絶対目標距離 [mm]
static float stop_cruise_mmps = 0.0f;     // STOP引数speed_mmps（巡航速度想定）[mm/s]

static constexpr float FINAL_APPROACH_SPEED_MMPS = 50.0f;  // STOP時の最終進入速度

// グローバルなクラスは、上から順に初期化される
ADC adc(get_shared_spi());
LED led;
WallSensor wall_sensor(adc);
IMU imu(get_shared_spi());
Motor motor;
Encoder encoder;
Sensors sensors(imu, wall_sensor, adc, encoder);
MotionController motion(motor, sensors);

hw_timer_t* high_speed_timer = NULL;
std::atomic<uint32_t> timer_ticks(0);

QueueHandle_t cmd_queue; // コマンドキュー
QueueHandle_t msg_queue; // Core0 -> Core1 メッセージキュー（Serial出力用）

// ---- Core1(loop) -> Core0(RealtimeTask) コマンド定義 ----
struct SetMotorSpeedCommand {
    int16_t right_speed; // mm/s（互換）
    int16_t left_speed;  // mm/s（互換）
};

struct ForwardCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

struct StopCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

enum CommandID : uint8_t {
    CMD_SET_MOTOR_SPEED = 0x01,
    CMD_FORWARD = 0x04,
    CMD_STOP = 0x05,
    CMD_RESET_DISTANCE = 0x06,
    CMD_RESET_ANGLE = 0x07,
};

struct Command {
    CommandID cmd_id;
    union {
        SetMotorSpeedCommand set_motor_speed;
        ForwardCommand forward;
        StopCommand stop;
    } parameter;
};

// ---- Core0 -> Core1 メッセージ定義（Serial出力用） ----
// Core0->Core1で渡すメッセージ（固定長; 末尾は必ず\0終端）
struct MsgLine {
    char text[64];
};

static inline void enqueue_msg_line(const char* s) {
    if (!msg_queue || !s) return;
    MsgLine m;
    size_t i = 0;
    for (; i < sizeof(m.text) - 1 && s[i] != '\0'; ++i) {
        m.text[i] = s[i];
    }
    m.text[i] = '\0';
    (void)xQueueSend(msg_queue, &m, 0);
}

void IRAM_ATTR onHighSpeedTimer() {
    timer_ticks.fetch_add(1, std::memory_order_relaxed);
}

uint32_t waitTick(uint32_t &last_tick) {
    uint32_t current_tick = timer_ticks.load(std::memory_order_relaxed);
    while(current_tick == last_tick) {
        // 待機
        asm volatile("nop");
        current_tick = timer_ticks.load(std::memory_order_relaxed);
    }
    uint32_t delta = current_tick - last_tick;
    last_tick = current_tick;
    return delta;
}

void Core0RealtimeTask(void* parameter) {
    // ハイスピードタイマー(1ms)の初期化
    high_speed_timer = timerBegin(1000); // 1000Hz = 1ms period
    timerAttachInterrupt(high_speed_timer, &onHighSpeedTimer);
    timerAlarm(high_speed_timer, 1, true, 0); // 1ms interval, auto-reload

    uint32_t last_tick = 0;
    uint32_t time_delta = 0;

    while(1){
        // 1msごとの待機
        time_delta = waitTick(last_tick);

        // センサーデータの更新
        sensors.update(time_delta);

        // コマンドキューの処理（loop() -> Core0）
        Command q;
        while (xQueueReceive(cmd_queue, &q, 0) == pdTRUE) {
            if (q.cmd_id == CMD_SET_MOTOR_SPEED) {
                const float vr = static_cast<float>(q.parameter.set_motor_speed.right_speed) / 1000.0f;
                const float vl = static_cast<float>(q.parameter.set_motor_speed.left_speed) / 1000.0f;

                // 手動MOTが来たらプロファイルは停止（競合回避）
                fwd_active = false;
                stop_active = false;

                target_vr_mps = vr;
                target_vl_mps = vl;
                motion.forward((vr + vl) * 0.5f, 0.0f);
            } else if (q.cmd_id == CMD_FORWARD) {
                // FWD: 加速して指定速度へ、距離到達でDONE（停止しない）
                fwd_active = true;
                stop_active = false;

                fwd_v_target_mmps = q.parameter.forward.speed_mmps;
                fwd_a_mmps2 = q.parameter.forward.accel_mmps2;

                const float now_dist = sensors.get_distance();
                fwd_goal_dist_mm = now_dist + q.parameter.forward.distance_mm;

                // 現在の指令速度（左右平均）を初期値に
                fwd_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;

                // 目標が今より遅いなら減速方向に
                if (fwd_v_target_mmps < fwd_v_cmd_mmps && fwd_a_mmps2 > 0) {
                    fwd_a_mmps2 = -fwd_a_mmps2;
                }

                if (fwd_a_mmps2 == 0.0f) {
                    fwd_v_cmd_mmps = fwd_v_target_mmps;
                }
            } else if (q.cmd_id == CMD_STOP) {
                // STOP: 指定距離で停止（必要に応じて50mm/sまで減速して進入）
                stop_active = true;
                stop_done_pending = false;
                fwd_active = false;

                // 引数の速度は「現在速度の想定」だが、ここでは現在の指令速度も併用
                stop_v_target_mmps = 0.0f;
                stop_a_mmps2 = fabsf(q.parameter.stop.accel_mmps2);
                stop_cruise_mmps = fabsf(q.parameter.stop.speed_mmps);

                const float now_dist = sensors.get_distance();
                stop_goal_dist_mm = now_dist + q.parameter.stop.distance_mm;

                // 初期速度: 現在指令速度を優先
                stop_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;
                if (stop_v_cmd_mmps <= 0.0f) {
                    stop_v_cmd_mmps = stop_cruise_mmps;
                }
            } else if (q.cmd_id == CMD_RESET_DISTANCE) {
                sensors.reset_distance();
                enqueue_msg_line("#distance reset\n");
            } else if (q.cmd_id == CMD_RESET_ANGLE) {
                sensors.reset_angle();
                enqueue_msg_line("#angle reset\n");
            }
        }

        const float dt_s = static_cast<float>(time_delta) / 1000.0f;

        // FWD 更新: 目標距離でDONE、停止しない
        if (fwd_active) {
            const float now_dist = sensors.get_distance();
            const float remain_mm = fwd_goal_dist_mm - now_dist;

            if (remain_mm <= 0.0f) {
                fwd_active = false;
                enqueue_msg_line("DONE\n");
            } else {
                float v_next = fwd_v_cmd_mmps;
                const float a_mag = fabsf(fwd_a_mmps2);

                if (a_mag > 1e-3f) {
                    if (fwd_v_target_mmps > fwd_v_cmd_mmps) {
                        v_next = fwd_v_cmd_mmps + a_mag * dt_s;
                        if (v_next > fwd_v_target_mmps) v_next = fwd_v_target_mmps;
                    } else if (fwd_v_target_mmps < fwd_v_cmd_mmps) {
                        v_next = fwd_v_cmd_mmps - a_mag * dt_s;
                        if (v_next < fwd_v_target_mmps) v_next = fwd_v_target_mmps;
                    }
                } else {
                    v_next = fwd_v_target_mmps;
                }

                fwd_v_cmd_mmps = v_next;
                const float v_cmd_mps = fwd_v_cmd_mmps / 1000.0f;
                target_vr_mps = v_cmd_mps;
                target_vl_mps = v_cmd_mps;
                motion.forward(v_cmd_mps, 0.0f);
            }
        }

        // STOP 更新: 指定距離で停止しDONE
        if (stop_active) {
            const float now_dist = sensors.get_distance();
            const float remain_mm = stop_goal_dist_mm - now_dist;

            // 目標距離に到達したら、まず停止指令を出し、次のループでDONEを返す
            if (remain_mm <= 0.0f) {
                stop_active = false;
                stop_done_pending = true;
                stop_v_cmd_mmps = 0.0f;

                target_vr_mps = 0.0f;
                target_vl_mps = 0.0f;
                motion.stop();
            } else {
                const float a_mag = stop_a_mmps2;
                const float v = stop_v_cmd_mmps;

                // まず 50mm/s まで減速する必要があるかを判断
                float dist_to_50 = 0.0f;
                if (a_mag > 1e-3f) {
                    const float dv2 = (v * v) - (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS);
                    dist_to_50 = dv2 > 0.0f ? (dv2 / (2.0f * a_mag)) : 0.0f;
                }

                // 次に 50mm/s から 0 まで止めるのに必要な距離
                float dist_50_to_0 = 0.0f;
                if (a_mag > 1e-3f) {
                    dist_50_to_0 = (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS) / (2.0f * a_mag);
                }

                float v_next = v;

                // 残距離が「50->0の距離」以下なら、優先して停止に向けて減速（50以下ならそのまま減速）
                if (remain_mm <= dist_50_to_0) {
                    if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
                    if (v_next < 0.0f) v_next = 0.0f;
                } else if (remain_mm <= (dist_to_50 + dist_50_to_0)) {
                    // そろそろ 50mm/s まで落とすフェーズ
                    if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
                    if (v_next < FINAL_APPROACH_SPEED_MMPS) v_next = FINAL_APPROACH_SPEED_MMPS;
                } else {
                    // まだ余裕がある: 今の速度維持（必要なら指定速度へ合わせる）
                    // STOPの引数 speed_mmps は「巡航速度想定」なので、現在がそれ以下なら軽く合わせる
                    const float cruise = stop_cruise_mmps;
                    if (cruise > 0.0f && v < cruise && a_mag > 1e-3f) {
                        v_next = v + a_mag * dt_s;
                        if (v_next > cruise) v_next = cruise;
                    }
                }

                stop_v_cmd_mmps = v_next;
                const float v_cmd_mps = stop_v_cmd_mmps / 1000.0f;

                target_vr_mps = v_cmd_mps;
                target_vl_mps = v_cmd_mps;
                if (v_cmd_mps == 0.0f) {
                    motion.stop();
                } else {
                    motion.forward(v_cmd_mps, 0.0f);
                }
            }
        }

        // STOPのDONE返却（停止指令後に一度だけ）
        if (stop_done_pending) {
            stop_done_pending = false;
            enqueue_msg_line("DONE\n");
        }

        // Motion controller update (speed PID)
        motion.update(time_delta);
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
    cmd_queue = xQueueCreate(16, sizeof(Command));
    if (cmd_queue == NULL) {
        Serial.printf("#Failed to create command queue!\n");
        while(1);
    }

    // Core0 -> Core1 メッセージキューの作成
    msg_queue = xQueueCreate(32, sizeof(MsgLine));
    if (msg_queue == NULL) {
        Serial.printf("#Failed to create message queue!\n");
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
        Serial.println("#Failed to create Core0 task!\n");
        while(1);
    }
    
    Serial.printf("#Core 0 realtime task started\n");
    Serial.printf("#System ready\n");
}
 
// Core1のループ関数
void loop() {
    // Core0からのメッセージ出力（Serial.printfはCore1側でのみ行う）
    {
        MsgLine m;
        while (msg_queue && xQueueReceive(msg_queue, &m, 0) == pdTRUE) {
            Serial.print(m.text);
        }
    }

    // センサーデータの読み取り例
    static unsigned long last_print = 0;
    if (millis() - last_print >= 100) {
        last_print = millis();
        
        float gyro = sensors.get_gyro_z();      // rad/s
        float vbatt = sensors.get_battery_voltage();
        uint16_t lf = sensors.get_lf();
        uint16_t ls = sensors.get_ls();
        uint16_t rs = sensors.get_rs();
        uint16_t rf = sensors.get_rf();
        uint16_t enc_r = sensors.get_right_wheel_angle();
        uint16_t enc_l = sensors.get_left_wheel_angle();
        float odo_dist = sensors.get_distance();
        float odo_ang = sensors.get_angle();    // rad
        
        // Note: gyro_z [rad/s], odo_ang [rad]
        Serial.printf("SEN,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%.2f,%.2f\n",
                      gyro, vbatt, lf, ls, rs, rf, enc_r, enc_l, odo_dist, odo_ang);
    }
    
    // UARTコマンド受信例
    if (Serial.available()) {
        // e.g. "MOT,100,100\n"
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("MOT")) {
            // モーター速度設定: MOT,100,100
            // 期待フォーマット: "MOT,<right_speed>,<left_speed>"
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1) {
                Command q;
                q.cmd_id = CMD_SET_MOTOR_SPEED;
                q.parameter.set_motor_speed.right_speed = cmd.substring(comma1 + 1, comma2).toInt();
                q.parameter.set_motor_speed.left_speed = cmd.substring(comma2 + 1).toInt();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#Motor: R=%d, L=%d\n",
                                  q.parameter.set_motor_speed.right_speed,
                                  q.parameter.set_motor_speed.left_speed);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid MOT format\n");
            }
        } else if (cmd.startsWith("WALL")) {
            // 壁センサLEDの有効/無効: WALL,1 または WALL,0
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int en = cmd.substring(comma1 + 1).toInt();
                bool enabled = (en != 0);
                wall_sensor.set_enabled(enabled);
                Serial.printf("#WallSensor enabled=%d\n", enabled ? 1 : 0);
            } else {
                Serial.printf("#Invalid WALL format\n");
            }
        } else if (cmd.startsWith("FWD,")) {
            // FWD: FWD,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_FORWARD;
                q.parameter.forward.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.forward.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.forward.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#FWD speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.forward.speed_mmps,
                                  q.parameter.forward.accel_mmps2,
                                  q.parameter.forward.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid FWD format\n");
            }
        } else if (cmd.startsWith("STOP,")) {
            // STOP: STOP,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_STOP;
                q.parameter.stop.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.stop.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.stop.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#STOP speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.stop.speed_mmps,
                                  q.parameter.stop.accel_mmps2,
                                  q.parameter.stop.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid STOP format\n");
            }
        } else if (cmd == "RDST") {
            // 距離リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_DISTANCE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RDST\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "RANG") {
            // 角度リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_ANGLE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RANG\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else {
            // デバッグ用: 不明コマンド
            Serial.printf("#Unknown cmd: %s\n", cmd.c_str());
        }
    }
    
    delay(1);
}
