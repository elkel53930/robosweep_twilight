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
    //   ODR = 416Hz (0110b), FS = ±1000dps (10b)
    const uint8_t gyro_config[2] = {0x11, 0x68};  // 0b01101000
    imu_transfer(r_buffer, gyro_config, 2);
    
    // CTRL1_XL: 加速度センサー設定 (オプション)
    //   ODR = 416Hz (0110b), FS = ±4g (10b)
    const uint8_t accel_config[2] = {0x10, 0x68};  // 0b01101000
    imu_transfer(r_buffer, accel_config, 2);
    
    // CTRL3_C: ブロックデータ更新を有効化
    const uint8_t ctrl3_config[2] = {0x12, 0x44};  // BDU bit + IF_INC (auto-increment)
    imu_transfer(r_buffer, ctrl3_config, 2);
    
    // CTRL6_C: ジャイロのハイパフォーマンスモードを有効化
    const uint8_t ctrl6_config[2] = {0x15, 0x00};  // High performance mode for gyro
    imu_transfer(r_buffer, ctrl6_config, 2);
    
    // CTRL7_G: ジャイロのローパスフィルタを有効化
    //   LPF1 enabled, cutoff = ODR/4 (約104Hz)
    const uint8_t ctrl7_config[2] = {0x16, 0x00};  // LPF1 enabled by default
    imu_transfer(r_buffer, ctrl7_config, 2);
    
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
    // LSM6DSR: レジスタ0x26=LSB, 0x27=MSB (リトルエンディアン)
    // r_buffer[0] = ダミー, r_buffer[1] = LSB, r_buffer[2] = MSB
    int16_t result = ((int16_t)r_buffer[2] << 8) | (int16_t)r_buffer[1];
    
    // デバッグ用: バイトオーダーを試すため、逆も試してみる
    // int16_t result_swap = ((int16_t)r_buffer[1] << 8) | (int16_t)r_buffer[2];
    // もし逆の方がノイズが少なければ、上の行をコメントアウトして下の行を使う
    
    return result;
}

float IMU::convert_gyro_z_to_radps(int16_t raw_value) {
    // 生データを物理値（rad/s）に変換
    // FS = ±1000dps の場合、感度は 35 mdps/LSB = 0.035 dps/LSB
    // rad/s = dps * pi/180
    return static_cast<float>(raw_value) * GYRO_SENSITIVITY_RADPS;
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
    delayMicroseconds(50); // より長いsetup time
    
    // データの送受信
    for (size_t i = 0; i < length; i++) {
        r_buffer[i] = spi.transfer(w_buffer[i]);
    }
    
    // より長いhold time
    delayMicroseconds(50);
    // CSピンをHIGHにしてデバイス非選択
    digitalWrite(IMU_CS, HIGH);
    
    // トランザクション終了
    spi.endTransaction();
}