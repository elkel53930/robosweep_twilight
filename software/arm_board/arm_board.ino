/*
 * arm_board.ino - Arduino Nano control board for robotic arm
 * 
 * Pin Configuration:
 * - D8, D9: PWM servo motors
 * - A4, A5: I2C current sensor (INA219)
 * - A1: Battery voltage monitor (10:1 voltage divider)
 * - D3: N-ch MOSFET PWM motor control
 */

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_INA219.h>

// Pin definitions
#define SERVO1_PIN 8
#define SERVO2_PIN 9
#define BATTERY_VOLTAGE_PIN A1
#define MOTOR_PWM_PIN 3

// Constants
#define VOLTAGE_DIVIDER_RATIO 10.0  // 10:1 voltage divider
#define ADC_REFERENCE_VOLTAGE 5.0   // Arduino Nano reference voltage
#define ADC_MAX_VALUE 1023.0        // 10-bit ADC

// Objects
Servo servo1;
Servo servo2;
Adafruit_INA219 ina219;

// Global variables
float batteryVoltage = 0.0;
float current_mA = 0.0;
float busVoltage = 0.0;
float power_mW = 0.0;
bool ina219Available = false;

// Serial communication variables
String inputString = "";
bool stringComplete = false;
unsigned long lastSensorSend = 0;
const unsigned long SENSOR_SEND_INTERVAL = 100;  // 100ms

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Arduino Nano Arm Board Initializing...");
  
  // Initialize servo motors
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Set servos to neutral position (90 degrees)
  servo1.write(90);
  servo2.write(90);
  Serial.println("Servos initialized");
  
  // Initialize PWM motor pin
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  
  // Set PWM frequency to 2kHz for motor control
  setupMotorPWM();
  
  analogWrite(MOTOR_PWM_PIN, 0);  // Start with motor off
  Serial.println("Motor PWM pin initialized (2kHz)");
  
  // Initialize battery voltage monitoring pin
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  Serial.println("Battery voltage monitor initialized");
  
  // Initialize I2C communication
  Wire.begin();
  Serial.println("I2C initialized");
  
  // Scan for I2C devices
  scanI2CDevices();
  
  // Initialize I2C current sensor
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip - continuing without current sensor");
    ina219Available = false;
  } else {
    Serial.println("INA219 current sensor initialized");
    // Calibration for INA219 (32V, 2A range)
    ina219.setCalibration_32V_2A();
    ina219Available = true;
  }
  
  Serial.println("Setup complete!");
  Serial.println("Commands: S1<angle> S2<angle> M<speed> (e.g., S1090, S2180, M050)");
  inputString.reserve(200);  // Reserve memory for input string
  delay(1000);
}

void loop() {
  // Handle serial communication
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Read sensor values
  readBatteryVoltage();
  readCurrentSensor();
  
  // Send sensor data every 100ms
  if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
    sendSensorData();
    lastSensorSend = millis();
  }
  
  delay(1);  // Small delay for stability
}

// Utility Functions

/**
 * Serial event handler
 */
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

/**
 * Process incoming commands
 * Commands: S1<angle> S2<angle> M<speed>
 * Examples: S1090, S2180, M050
 */
void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command.startsWith("S1")) {
    int angle = command.substring(2).toInt();
    setServo1Angle(angle);
    Serial.println("OK:S1:" + String(angle));
  }
  else if (command.startsWith("S2")) {
    int angle = command.substring(2).toInt();
    setServo2Angle(angle);
    Serial.println("OK:S2:" + String(angle));
  }
  else if (command.startsWith("M")) {
    int speed = command.substring(1).toInt();
    setMotorSpeedPercent(speed);
    Serial.println("OK:M:" + String(speed));
  }
  else if (command == "STATUS") {
    sendSensorData();
  }
  else if (command == "STOP") {
    emergencyStop();
    Serial.println("OK:STOP");
  }
  else {
    Serial.println("ERROR:UNKNOWN_COMMAND:" + command);
  }
}

/**
 * Send sensor data in structured format
 * Format: DATA:<battery_voltage>,<current_mA>,<power_mW>
 */
void sendSensorData() {
  Serial.print("DATA:");
  Serial.print(batteryVoltage, 2);
  Serial.print(",");
  Serial.print(current_mA, 1);
  Serial.print(",");
  Serial.println(power_mW, 1);
}

/**
 * Read battery voltage through voltage divider
 */
void readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_VOLTAGE_PIN);
  batteryVoltage = (adcValue * ADC_REFERENCE_VOLTAGE / ADC_MAX_VALUE) * VOLTAGE_DIVIDER_RATIO;
}

/**
 * Read current sensor values
 */
void readCurrentSensor() {
  if (ina219Available) {
    busVoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
  } else {
    busVoltage = 0.0;
    current_mA = 0.0;
    power_mW = 0.0;
  }
}

/**
 * Print all sensor values to serial (for debugging)
 */
void printSensorValues() {
  Serial.println("DEBUG:=== Sensor Readings ===");
  Serial.print("DEBUG:Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");
  
  if (ina219Available) {
    Serial.print("DEBUG:Bus Voltage: ");
    Serial.print(busVoltage, 2);
    Serial.println(" V");
    
    Serial.print("DEBUG:Current: ");
    Serial.print(current_mA, 1);
    Serial.println(" mA");
    
    Serial.print("DEBUG:Power: ");
    Serial.print(power_mW, 1);
    Serial.println(" mW");
  } else {
    Serial.println("DEBUG:INA219 not available - no current/power data");
  }
  
  Serial.println("DEBUG:========================");
}

/**
 * Control servo motor 1
 * @param angle: angle in degrees (0-180)
 */
void setServo1Angle(int angle) {
  angle = constrain(angle, 0, 180);
  servo1.write(angle);
}

/**
 * Control servo motor 2
 * @param angle: angle in degrees (0-180)
 */
void setServo2Angle(int angle) {
  angle = constrain(angle, 0, 180);
  servo2.write(angle);
}

/**
 * Control motor speed via PWM (0-255)
 * @param speed: speed value (0-255)
 */
void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(MOTOR_PWM_PIN, speed);
}

/**
 * Setup PWM frequency for motor control (2kHz)
 */
void setupMotorPWM() {
  // Timer2 settings for 2kHz PWM frequency
  // Fast PWM mode, non-inverting, prescaler = 32
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  // Fast PWM mode
  TCCR2B = _BV(CS21) | _BV(CS20);                  // Prescaler = 32
  // Frequency = 16MHz / (32 * 256) = 1.953kHz ≈ 2kHz
}

/**
 * Control motor speed in percentage (0-100%)
 * @param percent: speed percentage (0-100)
 */
void setMotorSpeedPercent(int percent) {
  percent = constrain(percent, 0, 100);
  int pwmValue = map(percent, 0, 100, 0, 255);
  analogWrite(MOTOR_PWM_PIN, pwmValue);
}

/**
 * Get battery voltage
 * @return: battery voltage in volts
 */
float getBatteryVoltage() {
  return batteryVoltage;
}

/**
 * Get current consumption
 * @return: current in mA
 */
float getCurrentmA() {
  return current_mA;
}

/**
 * Get power consumption
 * @return: power in mW
 */
float getPowermW() {
  return power_mW;
}

/**
 * Scan for I2C devices
 */
void scanI2CDevices() {
  Serial.println("Scanning for I2C devices...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print(deviceCount);
    Serial.println(" I2C device(s) found");
  }
  Serial.println();
}

/**
 * Emergency stop - stop all motors and servos
 */
void emergencyStop() {
  servo1.write(90);  // Neutral position
  servo2.write(90);  // Neutral position
  analogWrite(MOTOR_PWM_PIN, 0);  // Stop motor
  Serial.println("EMERGENCY STOP ACTIVATED");
}