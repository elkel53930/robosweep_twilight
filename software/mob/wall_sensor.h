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

#endif