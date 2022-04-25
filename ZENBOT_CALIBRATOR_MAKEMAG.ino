#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL SerialUSB
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL SerialUSB
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#else // Other boards when using DynamixelShield
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
const float DXL_PROTOCOL_VERSION = 2.0;

int sensorValue[5];
int servo1High = 2120; //Rotate to low note
int servo1Low = 1730; //Rotate to high note
int servo2High = 2210; // mallet up
int servo2Low = 1600; //mallet Down
int rotateServoPos;
int malletServoDownPos;
int malletServoUpPos;
int servo;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(9600);
  //while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  for (int i = 1; i < 8; i++) {
    dxl.ping(i);
    dxl.ping(i + 10);
  }

  for (int i = 1; i < 8; i++) {
    // Turn off torque when configuring items in EEPROM area
    dxl.torqueOff(i);
    dxl.torqueOff(i + 10);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.setOperatingMode(i + 10, OP_POSITION);
    dxl.torqueOn(i);
    dxl.torqueOn(i + 10);
  }

  for (int i = 1; i < 8; i++) {
    // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
    dxl.writeControlTableItem(PROFILE_VELOCITY, i, 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, i + 10, 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i + 10, 0);
    dxl.setGoalPosition(i + 10, 1550);
    dxl.setGoalPosition(i, 2048);
  }
  delay(500);
}

void loop() {
  checkAnalogPins();
  rotateMallet();
  bangTheDrum();
  Serial.print("  Servo:");
  Serial.println(servo);
  Serial.print("Rot:");
  Serial.print(rotateServoPos);
  Serial.print("   Down:");
  Serial.print(malletServoDownPos);
  Serial.print("   Up:");
  Serial.println(malletServoUpPos);
}

void checkAnalogPins() {
  for (int analogPin = 1; analogPin < 5; analogPin++) {
    sensorValue[analogPin] = analogRead(analogPin);
    if (sensorValue[analogPin] < 24) sensorValue[analogPin] = 24;
  }
}

void  rotateMallet() {
  rotateServoPos = map(sensorValue[1], 24, 1023, servo1Low, servo1High);
  dxl.setGoalPosition(servo, rotateServoPos);
}

void bangTheDrum() {
  malletServoDownPos = map(sensorValue[2], 24, 1023, 1900, servo2High);
  malletServoUpPos = map(sensorValue[3], 24, 1023, servo2Low, 1899);
  servo = map(sensorValue[4], 24, 1023, 1, 7);
  servo = constrain(servo, 1, 7);

  dxl.setGoalPosition(servo + 10, malletServoDownPos);
  delay(150);
  dxl.setGoalPosition(servo + 10, malletServoUpPos);
  delay(1000);
}
