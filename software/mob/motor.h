#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor();
    void begin();  // モーター初期化
    void set_right_motor(int16_t speed);  // 右モーター速度設定 (-255 to 255)
    void set_left_motor(int16_t speed);   // 左モーター速度設定 (-255 to 255)
    void stop();  // 両モーター停止
    
private:
    // PWMピン定数
    static constexpr int RIGHT_PWM_PIN = 47;
    static constexpr int LEFT_PWM_PIN = 3;
    
    // 方向制御ピン定数
    static constexpr int RIGHT_DIR_PIN = 48;  // CW/CCW制御
    static constexpr int LEFT_DIR_PIN = 46;   // CW/CCW制御
    
    // PWM設定
    static constexpr int PWM_FREQUENCY = 10000;  // 10kHz
    static constexpr int PWM_RESOLUTION = 8;     // 8bit (0-255)
    
    // ヘルパー関数
    void set_motor_speed(int16_t speed, int pwm_pin, int dir_pin);
};

#endif