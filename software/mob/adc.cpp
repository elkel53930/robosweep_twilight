#include <Arduino.h>
#include <SPI.h>
#include "adc.h"

ADC::ADC(SPIClass& spi_bus) : spi(spi_bus) {
    // ピンの初期化のみ行う
    pinMode(BATT_CS, OUTPUT);
	pinMode(LF_CS, OUTPUT);
	pinMode(LS_CS, OUTPUT);
	pinMode(RF_CS, OUTPUT);
	pinMode(RS_CS, OUTPUT);
    digitalWrite(BATT_CS, HIGH);
	digitalWrite(LF_CS, HIGH);
	digitalWrite(LS_CS, HIGH);
	digitalWrite(RF_CS, HIGH);
	digitalWrite(RS_CS, HIGH);
}

void ADC::batt_read(float &voltage) {
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2];
    digitalWrite(BATT_CS, LOW);
    // 同時送受信
    spi.transferBytes(tx_buf, rx_buf, sizeof(tx_buf));
    digitalWrite(BATT_CS, HIGH);
    spi.endTransaction();
    uint16_t v_raw = rx_buf[0] * 256 + rx_buf[1];
    v_raw = v_raw & 0x1fff;
    v_raw = v_raw / 2;
    voltage = ((float)v_raw) * 3.3 * 11.0 / 4095.0;
}

void ADC::lf_read(uint16_t &result) {
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2];
    digitalWrite(LF_CS, LOW);
    // 同時送受信
    spi.transferBytes(tx_buf, rx_buf, sizeof(tx_buf));
    digitalWrite(LF_CS, HIGH);
    spi.endTransaction();
    uint16_t v_raw = rx_buf[0] * 256 + rx_buf[1];
    v_raw = v_raw & 0x1fff;
    v_raw = v_raw / 2;
    result = v_raw;
}

void ADC::ls_read(uint16_t &result) {
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2];
    digitalWrite(LS_CS, LOW);
    // 同時送受信
    spi.transferBytes(tx_buf, rx_buf, sizeof(tx_buf));
    digitalWrite(LS_CS, HIGH);
    spi.endTransaction();
    uint16_t v_raw = rx_buf[0] * 256 + rx_buf[1];
    v_raw = v_raw & 0x1fff;
    v_raw = v_raw / 2;
    result = v_raw;
}

void ADC::rf_read(uint16_t &result) {
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2];
    digitalWrite(RF_CS, LOW);
    // 同時送受信
    spi.transferBytes(tx_buf, rx_buf, sizeof(tx_buf));
    digitalWrite(RF_CS, HIGH);
    spi.endTransaction();
    uint16_t v_raw = rx_buf[0] * 256 + rx_buf[1];
    v_raw = v_raw & 0x1fff;
    v_raw = v_raw / 2;
    result = v_raw;
}

void ADC::rs_read(uint16_t &result) {
    spi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3));
    uint8_t tx_buf[2] = {0xFF, 0xFF};
    uint8_t rx_buf[2];
    digitalWrite(RS_CS, LOW);
    // 同時送受信
    spi.transferBytes(tx_buf, rx_buf, sizeof(tx_buf));
    digitalWrite(RS_CS, HIGH);
    spi.endTransaction();
    uint16_t v_raw = rx_buf[0] * 256 + rx_buf[1];
    v_raw = v_raw & 0x1fff;
    v_raw = v_raw / 2;
    result = v_raw;
}
