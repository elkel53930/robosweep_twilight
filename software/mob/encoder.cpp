#include <Arduino.h>
#include <SPI.h>
#include "encoder.h"

Encoder::Encoder() : encoder_spi(FSPI) {
    // FSPIを使用（HSPIは既存のADC/IMUで使用中）
}

void Encoder::begin() {
    // CSピンを出力として設定
    pinMode(RIGHT_CS_PIN, OUTPUT);
    pinMode(LEFT_CS_PIN, OUTPUT);
    
    // CSピンを初期状態（HIGH：非選択）に設定
    digitalWrite(RIGHT_CS_PIN, HIGH);
    digitalWrite(LEFT_CS_PIN, HIGH);
    
    // 専用SPIバスの初期化
    encoder_spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
    
    Serial.println("Encoder controller initialized");
    Serial.print("SPI pins - MOSI: GPIO");
    Serial.print(MOSI_PIN);
    Serial.print(", MISO: GPIO");
    Serial.print(MISO_PIN);
    Serial.print(", SCK: GPIO");
    Serial.println(SCK_PIN);
    Serial.print("CS pins - Right: GPIO");
    Serial.print(RIGHT_CS_PIN);
    Serial.print(", Left: GPIO");
    Serial.println(LEFT_CS_PIN);
    Serial.print("SPI frequency: ");
    Serial.print(SPI_FREQUENCY / 1000);
    Serial.println(" kHz");
    
    // 初期読み取りテスト
    delay(100);
    uint16_t right_test = read_right_angle();
    uint16_t left_test = read_left_angle();
    Serial.print("Initial encoder test - Right: ");
    Serial.print(right_test);
    Serial.print(", Left: ");
    Serial.println(left_test);
}

uint16_t Encoder::read_right_angle() {
    return read_encoder(RIGHT_CS_PIN);
}

uint16_t Encoder::read_left_angle() {
    return read_encoder(LEFT_CS_PIN);
}

float Encoder::read_right_angle_degrees() {
    uint16_t raw_angle = read_right_angle();
    return raw_angle * DEGREES_PER_COUNT;
}

float Encoder::read_left_angle_degrees() {
    uint16_t raw_angle = read_left_angle();
    return raw_angle * DEGREES_PER_COUNT;
}

bool Encoder::read_both_angles(uint16_t &right_angle, uint16_t &left_angle) {
    right_angle = read_right_angle();
    left_angle = read_left_angle();
    
    // 両方とも有効な値かチェック（0xFFFFは通信エラー）
    return (right_angle != 0xFFFF && left_angle != 0xFFFF);
}

uint16_t Encoder::read_encoder(int cs_pin) {
    // AS5047の角度レジスタを読み取り（参考コードに基づく）
    // コマンド: 0x7FFE（角度読み取り）
    uint8_t w_buffer[2] = {0x7F, 0xFE};
    uint8_t r_buffer[2] = {0, 0};
    
    // SPI転送
    encoder_spi.beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE));
    
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    
    // バイト単位で転送
    r_buffer[0] = encoder_spi.transfer(w_buffer[0]);
    r_buffer[1] = encoder_spi.transfer(w_buffer[1]);
    
    digitalWrite(cs_pin, HIGH);
    encoder_spi.endTransaction();
    
    // 結果を結合（参考コードのconcat関数相当）
    uint16_t result = ((uint16_t)r_buffer[0] << 8) | r_buffer[1];
    result = result & 0x3FFF;  // 下位14bitのみ使用
    
    return result;
}