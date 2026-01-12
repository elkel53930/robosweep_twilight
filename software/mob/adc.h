#ifndef ADC_H
#define ADC_H

#include <Arduino.h>
#include <SPI.h>

class ADC{
public:
	ADC(SPIClass& spi_bus);
	void batt_read(float &voltage);
	void lf_read(uint16_t &result);
	void ls_read(uint16_t &result);
	void rf_read(uint16_t &result);
	void rs_read(uint16_t &result);
private:
	// ピン定数
	static constexpr int PIN_MISO = 9;
	static constexpr int PIN_MOSI = 10;
	static constexpr int PIN_SCK  = 11;
	static constexpr int BATT_CS   = 1;
	static constexpr int LF_CS = 16;
	static constexpr int LS_CS = 15;
	static constexpr int RF_CS = 40;
	static constexpr int RS_CS = 41;
	
	SPIClass& spi;
};

#endif