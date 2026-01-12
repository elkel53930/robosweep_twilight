#include <Arduino.h>
#include "motor.h"

Motor::Motor() {
    // コンストラクタでは何もしない（begin()で初期化）
}

void Motor::begin() {
    // 方向制御ピンを出力として設定
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    
    // 初期状態で両モーター停止
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN, LOW);
    
    // PWMチャンネルを設定し、ピンに接続
    ledcAttach(RIGHT_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(LEFT_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    
    // 初期状態でPWM出力を0に設定
    ledcWrite(RIGHT_PWM_PIN, 0);
    ledcWrite(LEFT_PWM_PIN, 0);
}

void Motor::set_right_motor(int16_t speed) {
    set_motor_speed(speed, RIGHT_PWM_PIN, RIGHT_DIR_PIN);
}

void Motor::set_left_motor(int16_t speed) {
    set_motor_speed(speed, LEFT_PWM_PIN, LEFT_DIR_PIN);
}

void Motor::stop() {
    // 両モーター停止
    ledcWrite(RIGHT_PWM_PIN, 0);
    ledcWrite(LEFT_PWM_PIN, 0);
    digitalWrite(RIGHT_DIR_PIN, LOW);
    digitalWrite(LEFT_DIR_PIN, LOW);
}

void Motor::set_motor_speed(int16_t speed, int pwm_pin, int dir_pin) {
    // 速度を-255から255の範囲に制限
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    
    // 方向と速度を設定
    if (speed >= 0) {
        // 正転 (CW)
        digitalWrite(dir_pin, HIGH);
        ledcWrite(pwm_pin, speed);
    } else {
        // 逆転 (CCW)
        digitalWrite(dir_pin, LOW);
        ledcWrite(pwm_pin, -speed);  // 負の値を正に変換
    }
}