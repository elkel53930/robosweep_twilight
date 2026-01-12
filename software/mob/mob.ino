#include <SPI.h>
#include "spi_manager.h"
#include "adc.h"
#include "led.h"
#include "wall_sensor.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"

// グローバルなクラスは、上から順に初期化される
ADC adc(get_shared_spi());
LED led;
WallSensor wall_sensor(adc);
IMU imu(get_shared_spi());
Motor motor;
Encoder encoder;

void setup() {
    Serial.begin(115200);
    delay(2000); // シリアル接続安定まで待つ
    Serial.println("=== System Start ===");
    
    init_shared_spi();  // 共有SPIの初期化
    Serial.println("Shared SPI initialized");
    
    motor.begin();
    Serial.println("Motor initialized");
    
    encoder.begin();
    Serial.println("Encoder initialized");
    
    imu.begin();
    Serial.println("IMU initialized");
    
    Serial.println("Setup completed. Starting encoder test...");
}
                                                                                                                                                                                        
void loop() {
    // エンコーダーテスト
    uint16_t right_angle, left_angle;
    
    encoder.read_both_angles(right_angle, left_angle);
    // 生値表示
    Serial.print("Encoder Raw - Right: ");
    Serial.print(right_angle);
    Serial.print(" (0x");
    Serial.print(right_angle, HEX);
    Serial.print("), Left: ");
    Serial.print(left_angle);
    Serial.print(" (0x");
    Serial.print(left_angle, HEX);
    Serial.println(")");
    
    // 角度（度）表示
    float right_degrees = encoder.read_right_angle_degrees();
    float left_degrees = encoder.read_left_angle_degrees();
    Serial.print("Encoder Degrees - Right: ");
    Serial.print(right_degrees, 2);
    Serial.print("°, Left: ");
    Serial.print(left_degrees, 2);
    Serial.println("°");
    
    // IMUテスト（簡単な表示）
    Serial.print("IMU WHO_AM_I: 0x");
    Serial.print(imu.read_who_am_i(), HEX);
    Serial.print(", Gyro Z: ");
    Serial.println(imu.read_gyro_z());
    
    Serial.println("---");
    delay(500);  // 500ms間隔
}
