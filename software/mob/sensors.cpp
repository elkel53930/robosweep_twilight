#include "sensors.h"

Sensors::Sensors(IMU& imu, WallSensor& wall_sensor, ADC& adc, Encoder& encoder)
    : imu_(imu), wall_sensor_(wall_sensor), adc_(adc), encoder_(encoder),
      gyro_z_(0.0f), lf_(0), rf_(0), ls_(0), rs_(0),
      battery_voltage_(0.0f), right_wheel_angle_(0), left_wheel_angle_(0) {
}

void Sensors::update() {
    // IMUデータの読み取り
    int16_t raw_gyro_z = imu_.read_gyro_z();
    float gyro_z_dps = imu_.convert_gyro_z_to_dps(raw_gyro_z);
    gyro_z_.store(gyro_z_dps, std::memory_order_relaxed);
    
    // 壁センサー値の読み取り
    uint16_t lf_value = wall_sensor_.lf();
    uint16_t rf_value = wall_sensor_.rf();
    uint16_t ls_value = wall_sensor_.ls();
    uint16_t rs_value = wall_sensor_.rs();
    lf_.store(lf_value, std::memory_order_relaxed);
    rf_.store(rf_value, std::memory_order_relaxed);
    ls_.store(ls_value, std::memory_order_relaxed);
    rs_.store(rs_value, std::memory_order_relaxed);
    
    // バッテリー電圧の読み取り
    float voltage;
    adc_.batt_read(voltage);
    battery_voltage_.store(voltage, std::memory_order_relaxed);
    
    // エンコーダー角度の読み取り（生値）
    uint16_t r_angle = encoder_.read_right_angle();
    uint16_t l_angle = encoder_.read_left_angle();
    right_wheel_angle_.store(r_angle, std::memory_order_relaxed);
    left_wheel_angle_.store(l_angle, std::memory_order_relaxed);
}

float Sensors::get_gyro_z() const {
    return gyro_z_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_lf() const {
    return lf_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_rf() const {
    return rf_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_ls() const {
    return ls_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_rs() const {
    return rs_.load(std::memory_order_relaxed);
}

float Sensors::get_battery_voltage() const {
    return battery_voltage_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_right_wheel_angle() const {
    return right_wheel_angle_.load(std::memory_order_relaxed);
}

uint16_t Sensors::get_left_wheel_angle() const {
    return left_wheel_angle_.load(std::memory_order_relaxed);
}
