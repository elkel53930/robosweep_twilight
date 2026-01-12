#include <SPI.h>
#include "spi_manager.h"
#include "adc.h"
#include "led.h"
#include "wall_sensor.h"
#include "imu.h"
#include "motor.h"

// グローバルなクラスは、上から順に初期化される
ADC adc(get_shared_spi());
LED led;
WallSensor wall_sensor(adc);
IMU imu(get_shared_spi());
Motor motor;

void setup() {
    Serial.begin(115200);
    delay(100); // シリアル接続安定まで待つ
    init_shared_spi();  // 共有SPIの初期化
    motor.begin();
    imu.begin();
}
                                                                                                                                                                                        
void loop() {
	
	Serial.printf("WHO_AM_I is");
	Serial.println(imu.read_who_am_i(), HEX);

	Serial.printf("Gyro Z:");
	Serial.println(imu.read_gyro_z());

	delay(50);
}
