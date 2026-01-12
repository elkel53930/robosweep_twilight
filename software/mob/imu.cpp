#include <Arduino.h>
#include <SPI.h>
#include "imu.h"

IMU::IMU(SPIClass& spi_bus) : spi(spi_bus) {
    // CSピンを出力として設定
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);  // 初期状態でHIGH（非選択）
}

bool IMU::begin() {
    uint8_t r_buffer[2] = {0x00, 0x00};
    
    // LSM6DSRは直接制御レジスタに書き込み可能
    // CTRL3_C: ソフトウェアリセットを実行
    const uint8_t reset_cmd[2] = {0x12, 0x01};  // SW_RESET bit
    imu_transfer(r_buffer, reset_cmd, 2);
    
    // リセット完了を待つ
    delay(50);
    
    // CTRL2_G: ジャイロスコープ設定
    //   ODR = 6.66kHz (1010b), FS = ±2000dps (11b), High Performance (0b)
    const uint8_t gyro_config[2] = {0x11, 0xAC};  // 0b10101100
    imu_transfer(r_buffer, gyro_config, 2);
    
    // CTRL1_XL: 加速度センサー設定 (オプション)
    //   ODR = 6.66kHz (1010b), FS = ±4g (10b), High Performance (0b)
    const uint8_t accel_config[2] = {0x10, 0xA8};  // 0b10101000
    imu_transfer(r_buffer, accel_config, 2);
    
    // CTRL3_C: ブロックデータ更新を有効化
    const uint8_t ctrl3_config[2] = {0x12, 0x40};  // BDU bit
    imu_transfer(r_buffer, ctrl3_config, 2);
    
    // 初期化後に少し待つ
    delay(100);
    return true;
}

int16_t IMU::read_gyro_z() {
    // Read OUTZ_L_G (26h) and OUTZ_H_G (27h)
    const uint8_t w_buffer[3] = {0xa6, 0xff, 0xff}; // Read 2 bytes from 0x26
    uint8_t r_buffer[3] = {0, 0, 0};
    imu_transfer(r_buffer, w_buffer, 3);
    
    // 受信データからZ軸ジャイロデータを構成
    int16_t result = ((int16_t)r_buffer[2] << 8) | (int16_t)r_buffer[1];
    
    return result;
}

uint8_t IMU::read_who_am_i() {
    // WHO_AM_I レジスタを読み取り
    const uint8_t w_buffer[2] = {0x8F, 0xFF}; // 0x0F with read bit (0x80)
    uint8_t r_buffer[2] = {0, 0};

	imu_transfer(r_buffer, w_buffer, 2);
    
    return r_buffer[1]; // 2番目のバイトが実際のデータ
}

uint8_t IMU::read_status() {
    // ステータスレジスタを読み取り
    const uint8_t w_buffer[2] = {0x9E, 0xFF}; // 0x1E with read bit (0x80)
    uint8_t r_buffer[2] = {0, 0};
    imu_transfer(r_buffer, w_buffer, 2);
    
    return r_buffer[1]; // 2番目のバイトが実際のデータ
}


void IMU::imu_transfer(uint8_t* r_buffer, const uint8_t* w_buffer, size_t length) {
    // SPI設定でトランザクション開始
    // ST社のIMUは通常MODE3、低めのクロック周波数で試す
    spi.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    
    // CSピンをLOWにしてデバイス選択
    digitalWrite(IMU_CS, LOW);
    delayMicroseconds(10); // より長いsetup time
    
    // データの送受信
    for (size_t i = 0; i < length; i++) {
        r_buffer[i] = spi.transfer(w_buffer[i]);
    }
    
    // より長いhold time
    delayMicroseconds(10);
    // CSピンをHIGHにしてデバイス非選択
    digitalWrite(IMU_CS, HIGH);
    
    // トランザクション終了
    spi.endTransaction();
}