#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <SPI.h>

class IMU {
public:
    IMU(SPIClass& spi_bus);
    bool begin();  // IMU初期化
    int16_t read_gyro_z();  // Z軸ジャイロデータ読み取り
    uint8_t read_who_am_i();  // WHO_AM_Iレジスタ読み取り（デバッグ用）
    uint8_t read_status();   // ステータスレジスタ読み取り
    
private:
    SPIClass& spi;
    
    // IMUのCSピン定数
    static constexpr int IMU_CS = 12;  // CSピンを適切なピンに設定してください
    
    // レジスタアドレス定数
    static constexpr uint8_t WHO_AM_I = 0x0F;
    static constexpr uint8_t WHO_AM_I_VALUE = 0x6B;  // LSM6DSRのWHO_AM_I値
    static constexpr uint8_t FUNC_CFG_ACCESS = 0x01;
    static constexpr uint8_t CTRL1_XL = 0x10;  // 加速度センサー制御
    static constexpr uint8_t CTRL2_G = 0x11;   // ジャイロスコープ制御
    static constexpr uint8_t CTRL3_C = 0x12;   // 共通制御
    static constexpr uint8_t STATUS_REG = 0x1E; // ステータスレジスタ
    static constexpr uint8_t OUTZ_L_G = 0x26;
    static constexpr uint8_t OUTZ_H_G = 0x27;
    
    // SPI通信ヘルパー関数
    void imu_transfer(uint8_t* r_buffer, const uint8_t* w_buffer, size_t length);
};

#endif