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
    
    // オドメトリ関連
    float get_distance() const;              // 移動距離（mm）
    float get_angle() const;                 // 姿勢角度（度）
    void reset_distance();                   // 距離リセット
    void reset_angle();                      // 角度リセット
    
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
    
    // オドメトリ用変数
    std::atomic<float> distance_;       // 累積移動距離（mm）
    std::atomic<float> angle_;          // 累積姿勢角度（度）
    uint16_t prev_right_angle_;         // 前回の右エンコーダ値
    uint16_t prev_left_angle_;          // 前回の左エンコーダ値
    
    // ロボット物理パラメータ
    static constexpr float WHEEL_DIAMETER = 23.4f;     // ホイール直径（mm）
    static constexpr float GEAR_RATIO = 41.0f / 20.0f; // エンコーダ:ホイールのギア比
    static constexpr float ENCODER_RESOLUTION = 16384.0f; // エンコーダ分解能（14bit）
    static constexpr float WHEEL_BASE = 50.0f;         // 車輪間距離（mm）※要調整
    static constexpr float SAMPLE_TIME = 0.001f;       // サンプリング周期（秒）= 1ms
    
    // エンコーダカウントから移動距離への変換係数（mm/count）
    static constexpr float COUNT_TO_MM = (WHEEL_DIAMETER * 3.14159265359f) / (ENCODER_RESOLUTION * GEAR_RATIO);
};

#endif
