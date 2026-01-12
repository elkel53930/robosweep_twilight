#include "led.h"

LED::LED() {
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(RED_PIN, OUTPUT);
}

void LED::green_on() {
	digitalWrite(GREEN_PIN, LOW);
}

void LED::green_off() {
	digitalWrite(GREEN_PIN, HIGH);
}

void LED::red_on() {
	digitalWrite(RED_PIN, LOW);
}
void LED::red_off() {
	digitalWrite(RED_PIN, HIGH);
}