#include "wall_sensor.h"

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
	enabled_ = true;
}

void WallSensor::set_enabled(bool enabled) {
	enabled_ = enabled;
	// 無効化時は確実にOFF
	if (!enabled_) {
		digitalWrite(LF_EN, LOW);
		digitalWrite(LS_EN, LOW);
		digitalWrite(RF_EN, LOW);
		digitalWrite(RS_EN, LOW);
	}
}

bool WallSensor::is_enabled() const {
	return enabled_;
}

uint16_t WallSensor::lf() {
	if (!enabled_) return 0;
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
	if (!enabled_) return 0;
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
	if (!enabled_) return 0;
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
	if (!enabled_) return 0;
	uint16_t off;
	adc_.rs_read(off);
	digitalWrite(RS_EN, HIGH);
	ets_delay_us(EN_DELAY_US);
	uint16_t on;
	adc_.rs_read(on);
	digitalWrite(RS_EN, LOW);
	return on - off;
}
