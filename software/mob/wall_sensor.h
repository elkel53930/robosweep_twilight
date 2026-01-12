#ifndef WALL_SENSOR_H
#define WALL_SENSOR_H

#include <Arduino.h>
#include "adc.h"

class WallSensor {
public:
	WallSensor(ADC& adc);
	uint16_t lf();
	uint16_t rf();
	uint16_t ls();
	uint16_t rs();
private:
	ADC& adc_;

	// 定数
	static constexpr uint16_t LF_EN = 6;
	static constexpr uint16_t LS_EN = 7;
	static constexpr uint16_t RF_EN = 2;
	static constexpr uint16_t RS_EN = 42;

	static constexpr uint32_t EN_DELAY_US = 100;
};

WallSensor::WallSensor(ADC& adc) : adc_(adc) {
	// センサーイネーブルピンを出力として設定
	pinMode(LF_EN, OUTPUT);
	pinMode(LS_EN, OUTPUT);
	pinMode(RF_EN, OUTPUT);
	pinMode(RS_EN, OUTPUT);
	// 初期状態でセンサーをOFF
	digitalWrite(LF_EN, LOW);
	digitalWrite(LS_EN, LOW);
	digitalWrite(RF_EN, LOW);
	digitalWrite(RS_EN, LOW);
}

uint16_t WallSensor::lf() {
	uint16_t off;
	adc_.lf_read(off);
	digitalWrite(LF_EN, HIGH);
	ets_delay_us(EN_DELAY_US);
	uint16_t on;
	adc_.lf_read(on);
	digitalWrite(LF_EN, LOW);
	return on - off;
}

uint16_t WallSensor::rf() {
	uint16_t off;
	adc_.rf_read(off);
	digitalWrite(RF_EN, HIGH);
	ets_delay_us(EN_DELAY_US);
	uint16_t on;
	adc_.rf_read(on);
	digitalWrite(RF_EN, LOW);
	return on - off;
}

uint16_t WallSensor::ls() {
	uint16_t off;
	adc_.ls_read(off);
	digitalWrite(LS_EN, HIGH);
	ets_delay_us(EN_DELAY_US);
	uint16_t on;
	adc_.ls_read(on);
	digitalWrite(LS_EN, LOW);
	return on - off;
}

uint16_t WallSensor::rs() {
	uint16_t off;
	adc_.rs_read(off);
	digitalWrite(RS_EN, HIGH);
	ets_delay_us(EN_DELAY_US);
	uint16_t on;
	adc_.rs_read(on);
	digitalWrite(RS_EN, LOW);
	return on - off;
}

#endif