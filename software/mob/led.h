#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED{
public:
	LED();
	void green_on();
	void green_off();
	void red_on();
	void red_off();
private:
	static constexpr uint16_t GREEN_PIN = 4;
	static constexpr uint16_t RED_PIN = 5;
};

#endif