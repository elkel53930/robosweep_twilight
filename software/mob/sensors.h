#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <atomic>
#include "imu.h"
#include "wall_sensor.h"
#include "adc.h"
#include "encoder.h"

class Sensors {
public:
    Sensors(IMU& imu, WallSensor& wall_sensor, ADC& adc, Encoder& encoder);
    
    // Core0から呼び出される関数: センサー値を読み取ってatomic変数に格納
    void update();
    
    // Core1のloop()から呼び出される関数: 格納された値を読み出す
    float get_gyro_z() const;           // Z軸ジャイロ角速度（度/秒）
    uint16_t get_lf() const;            // 左前壁センサー値
    uint16_t get_rf() const;            // 右前壁センサー値
    uint16_t get_ls() const;            // 左側壁センサー値
    uint16_t get_rs() const;            // 右側壁センサー値
    float get_battery_voltage() const;  // バッテリー電圧
    uint16_t get_right_wheel_angle() const;  // 右車輪角度（生値: 0-16383）
    uint16_t get_left_wheel_angle() const;   // 左車輪角度（生値: 0-16383）
    
private:
    // センサーへの参照
    IMU& imu_;
    WallSensor& wall_sensor_;
    ADC& adc_;
    Encoder& encoder_;
    
    // Atomic変数でセンサーデータを保持
    std::atomic<float> gyro_z_;         // Z軸ジャイロ角速度（度/秒）
    std::atomic<uint16_t> lf_;          // 左前壁センサー値
    std::atomic<uint16_t> rf_;          // 右前壁センサー値
    std::atomic<uint16_t> ls_;          // 左側壁センサー値
    std::atomic<uint16_t> rs_;          // 右側壁センサー値
    std::atomic<float> battery_voltage_; // バッテリー電圧
    std::atomic<uint16_t> right_wheel_angle_; // 右車輪角度（生値: 0-16383）
    std::atomic<uint16_t> left_wheel_angle_;  // 左車輪角度（生値: 0-16383）
};

#endif
