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
#define VOLTAGE_DIVIDER_RATIO 11.0  // 10:1 voltage divider
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
const unsigned long SENSOR_SEND_INTERVAL = 25;  // 25ms (40Hz)

// Servo control variables
float servo1_current_angle = 90.0;
float servo1_target_angle = 90.0;
int servo1_speed = 180;  // degrees per second (default: max speed)
float servo2_current_angle = 90.0;
float servo2_target_angle = 90.0;
int servo2_speed = 180;  // degrees per second (default: max speed)
unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 5;  // 5ms = 200Hz update rate

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
  volatile int dummy;
  for(int i=0; i!=3200; i++) { dummy = i; }
}

void loop() {
  // Handle serial communication
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Update servo positions with speed control
  if (millis() - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    updateServoPositions();
    lastServoUpdate = millis();
  }
  
  // Read sensor values
  readBatteryVoltage();
  readCurrentSensor();
  
  // Send sensor data every 100ms
  if (millis() - lastSensorSend >= SENSOR_SEND_INTERVAL) {
    sendSensorData();
    lastSensorSend = millis();
  }
  
  // Small delay for stability (using empty loop instead of delay)
  volatile int dummy;
  for(int i=0; i!=2400; i++) { dummy = i; }  // approximately 1ms with assignment
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
 * Commands: S1<angle> S2<angle> M<speed> S1S<speed> S2S<speed> S1A<angle>,<speed> S2A<angle>,<speed>
 * Examples: S1090, S2180, M050, S1S030, S2S060, S1A090,030, S2A045,060
 */
void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  // デバッグ用: 受信したコマンドを表示
  Serial.println("DEBUG:Received command: '" + command + "'");
  
  if (command.startsWith("S1A")) {
    // S1A<angle>,<speed> - Set servo1 angle with speed
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      int angle = command.substring(3, commaIndex).toInt();
      int speed = command.substring(commaIndex + 1).toInt();
      setServo1AngleWithSpeed(angle, speed);
      Serial.println("OK:S1A:" + String(angle) + "," + String(speed));
    } else {
      Serial.println("ERROR:S1A_FORMAT");
    }
  }
  else if (command.startsWith("S2A")) {
    // S2A<angle>,<speed> - Set servo2 angle with speed
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      int angle = command.substring(3, commaIndex).toInt();
      int speed = command.substring(commaIndex + 1).toInt();
      setServo2AngleWithSpeed(angle, speed);
      Serial.println("OK:S2A:" + String(angle) + "," + String(speed));
    } else {
      Serial.println("ERROR:S2A_FORMAT");
    }
  }
  else if (command.startsWith("S1S")) {
    // S1S<speed> - Set servo1 default speed
    int speed = command.substring(3).toInt();
    setServo1Speed(speed);
    Serial.println("OK:S1S:" + String(speed));
  }
  else if (command.startsWith("S2S")) {
    // S2S<speed> - Set servo2 default speed
    int speed = command.substring(3).toInt();
    setServo2Speed(speed);
    Serial.println("OK:S2S:" + String(speed));
  }
  else if (command.startsWith("S1")) {
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
 * Update servo positions with speed control
 */
void updateServoPositions() {
  // Update servo1 position
  if (servo1_current_angle != servo1_target_angle) {
    float step_size = (float)servo1_speed * SERVO_UPDATE_INTERVAL / 1000.0;  // degrees per update
    
    if (abs(servo1_target_angle - servo1_current_angle) <= step_size) {
      servo1_current_angle = servo1_target_angle;
    } else if (servo1_target_angle > servo1_current_angle) {
      servo1_current_angle += step_size;
    } else {
      servo1_current_angle -= step_size;
    }
    
    servo1.write((int)servo1_current_angle);
  }
  
  // Update servo2 position
  if (servo2_current_angle != servo2_target_angle) {
    float step_size = (float)servo2_speed * SERVO_UPDATE_INTERVAL / 1000.0;  // degrees per update
    
    if (abs(servo2_target_angle - servo2_current_angle) <= step_size) {
      servo2_current_angle = servo2_target_angle;
    } else if (servo2_target_angle > servo2_current_angle) {
      servo2_current_angle += step_size;
    } else {
      servo2_current_angle -= step_size;
    }
    
    servo2.write((int)servo2_current_angle);
  }
}

/**
 * Control servo motor 1 (immediate)
 * @param angle: angle in degrees (0-180)
 */
void setServo1Angle(int angle) {
  angle = constrain(angle, 0, 180);
  servo1_current_angle = (float)angle;
  servo1_target_angle = (float)angle;
  servo1.write(angle);
}

/**
 * Control servo motor 2 (immediate)
 * @param angle: angle in degrees (0-180)
 */
void setServo2Angle(int angle) {
  angle = constrain(angle, 0, 180);
  servo2_current_angle = (float)angle;
  servo2_target_angle = (float)angle;
  servo2.write(angle);
}

/**
 * Control servo motor 1 with speed
 * @param angle: target angle in degrees (0-180)
 * @param speed: speed in degrees per second (1-180)
 */
void setServo1AngleWithSpeed(int angle, int speed) {
  angle = constrain(angle, 0, 180);
  speed = constrain(speed, 1, 180);
  servo1_target_angle = (float)angle;
  servo1_speed = speed;
  
  // デバッグ情報を追加
  Serial.print("DEBUG:S1 current=");
  Serial.print(servo1_current_angle);
  Serial.print(" target=");
  Serial.print(servo1_target_angle);
  Serial.print(" speed=");
  Serial.println(servo1_speed);
}

/**
 * Control servo motor 2 with speed
 * @param angle: target angle in degrees (0-180)
 * @param speed: speed in degrees per second (1-180)
 */
void setServo2AngleWithSpeed(int angle, int speed) {
  angle = constrain(angle, 0, 180);
  speed = constrain(speed, 1, 180);
  servo2_target_angle = (float)angle;
  servo2_speed = speed;
  
  // デバッグ情報を追加
  Serial.print("DEBUG:S2 current=");
  Serial.print(servo2_current_angle);
  Serial.print(" target=");
  Serial.print(servo2_target_angle);
  Serial.print(" speed=");
  Serial.println(servo2_speed);
}

/**
 * Set servo1 default speed
 * @param speed: speed in degrees per second (1-180)
 */
void setServo1Speed(int speed) {
  speed = constrain(speed, 1, 180);
  servo1_speed = speed;
}

/**
 * Set servo2 default speed
 * @param speed: speed in degrees per second (1-180)
 */
void setServo2Speed(int speed) {
  speed = constrain(speed, 1, 180);
  servo2_speed = speed;
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