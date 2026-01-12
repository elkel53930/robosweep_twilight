#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <SPI.h>

class Encoder {
public:
    Encoder();
    void begin();  // エンコーダー初期化
    uint16_t read_right_angle();  // 右エンコーダー角度読み取り (0-16383)
    uint16_t read_left_angle();   // 左エンコーダー角度読み取り (0-16383)
    float read_right_angle_degrees();  // 右エンコーダー角度読み取り (度)
    float read_left_angle_degrees();   // 左エンコーダー角度読み取り (度)
    bool read_both_angles(uint16_t &right_angle, uint16_t &left_angle);  // 両方同時読み取り
    
private:
    // 専用SPIバス（既存のSPIと競合回避）
    SPIClass encoder_spi;
    
    // SPI通信ピン定数
    static constexpr int MOSI_PIN = 17;
    static constexpr int MISO_PIN = 18;
    static constexpr int SCK_PIN = 8;
    static constexpr int RIGHT_CS_PIN = 20;
    static constexpr int LEFT_CS_PIN = 19;
    
    // AS5047設定（参考コードに基づく）
    static constexpr uint32_t SPI_FREQUENCY = 10000000; // 10MHz
    static constexpr uint8_t SPI_MODE = SPI_MODE1;      // AS5047はMODE1
    
    // AS5047分解能
    static constexpr uint16_t ENCODER_RESOLUTION = 16384;  // 14bit = 16384 counts
    static constexpr float DEGREES_PER_COUNT = 360.0f / ENCODER_RESOLUTION;
    
    // ヘルパー関数
    uint16_t read_encoder(int cs_pin);
};

#endif