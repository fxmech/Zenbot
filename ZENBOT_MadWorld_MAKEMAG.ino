//ZENBOT 14-Servo Tongue Drum Player
//David Covarrubias
//fxmech@cammotion.net

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
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#else // Other boards when using DynamixelShield
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
const float DXL_PROTOCOL_VERSION = 2.0;

const int buttonPin = 16; //built in button on OpenCM Expansion Board
int buttonState = 0;

int malletRotateLowNote[8] = {0, 1975, 1975, 1975, 1975, 1975, 1975, 1997};//low octave
int malletRotateHighNote[8] = {0, 1776, 1776, 1795, 1730, 1753, 1800, 1731};//high octave
int malletUp = 1625;
int malletDown = 2121;

int reorder[8] = {0, 1, 3, 2, 7, 5, 4, 6}; 

int a;
int startFrame = 0;
int endFrame = 15;
int time2Move = 150; //millis that mallet spends dropping before it reverses direction

// 0= no note  1=low note loud  2= high note loud
int notesC[160] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int notesD[160] = {1, 0, 1, 1, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1,    0, 0, 1, 1, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,   0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,    2, 0, 1, 2, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,    1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0};
int notesE[160] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int notesF[160] = {0, 0, 1, 0, 2, 2, 1, 0, 1, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,    0, 0, 1, 0, 2, 2, 1, 0, 1, 0, 0, 0, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,   0, 0, 1, 0, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,    2, 2, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 2, 1, 0, 2, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int notesG[160] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 2, 0, 0, 1, 0, 2, 2, 0, 2, 1, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 1, 2, 0, 0, 1, 0, 2, 2, 0, 2, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 0, 2, 1, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 1, 0, 2, 2,    0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 0, 2, 1, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 2, 0, 0, 2, 2,    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 1, 0, 0, 0};
int notesA[160] = {0, 0, 1, 0, 0, 0, 1, 0, 2, 2, 1, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 1, 0, 0, 0, 1, 0, 2, 2, 1, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 1, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 2, 0, 0, 1, 0, 0, 0, 1, 0,    0, 0, 1, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0,    2, 2, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int notesB[160] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 2, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 1, 0, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 2, 2, 0, 0,    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  pinMode(buttonPin, INPUT); //Start Button Pin

  // Use UART port of DYNAMIXEL Shield to debug.
  //DEBUG_SERIAL.begin(115200);
  //while(!DEBUG_SERIAL);
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  for (int i = 1; i < 8; i++) {
    dxl.ping(i);
    dxl.ping(i + 10);
    delay(100);
  }

  // Turn off torque when configuring items in EEPROM area
  for (int i = 1; i < 8; i++) {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.torqueOn(i);
    dxl.torqueOff(i + 10);
    dxl.setOperatingMode(i + 10, OP_POSITION);
    dxl.torqueOn(i + 10);
    delay(100);
  }

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  for (int i = 1; i < 8; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, i, 0);
    dxl.writeControlTableItem(PROFILE_VELOCITY, i + 10, 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i, 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, i + 10, 0);
    dxl.setGoalPosition(i, 2024);
    dxl.setGoalPosition(i + 10, 1750);
    delay(100);
  }
  delay(500);
  startAnimation();
}
void playnotesA(int del) {
  startFrame = 0;
  endFrame = 31;

  for (int i = startFrame; i < endFrame + 1; i++) {
    // STRIKE NOTE
    if (notesC[i] == 1 || notesC[i] == 2)  dxl.setGoalPosition(11, malletDown);
    if (notesD[i] == 1 || notesD[i] == 2)  dxl.setGoalPosition(12, malletDown);
    if (notesE[i] == 1 || notesE[i] == 2)  dxl.setGoalPosition(13, malletDown);
    if (notesF[i] == 1 || notesF[i] == 2)  dxl.setGoalPosition(14, malletDown);
    if (notesG[i] == 1 || notesG[i] == 2)  dxl.setGoalPosition(15, malletDown);
    if (notesA[i] == 1 || notesA[i] == 2)  dxl.setGoalPosition(16, malletDown);
    if (notesB[i] == 1 || notesB[i] == 2)  dxl.setGoalPosition(17, malletDown);
    delay(time2Move);

    //PREP FOR NEXT NOTE
    if (i + 1 >= endFrame + 1) a = startFrame; else a = i + 1;
    if (notesC[a] == 1)dxl.setGoalPosition(1, malletRotateLowNote[1]); else dxl.setGoalPosition(1, malletRotateHighNote[1]);
    if (notesD[a] == 1)dxl.setGoalPosition(2, malletRotateLowNote[2]); else dxl.setGoalPosition(2, malletRotateHighNote[2]);
    if (notesE[a] == 1)dxl.setGoalPosition(3, malletRotateLowNote[3]); else dxl.setGoalPosition(3, malletRotateHighNote[3]);
    if (notesF[a] == 1)dxl.setGoalPosition(4, malletRotateLowNote[4]); else dxl.setGoalPosition(4, malletRotateHighNote[4]);
    if (notesG[a] == 1)dxl.setGoalPosition(5, malletRotateLowNote[5]); else dxl.setGoalPosition(5, malletRotateHighNote[5]);
    if (notesA[a] == 1)dxl.setGoalPosition(6, malletRotateLowNote[6]); else dxl.setGoalPosition(6, malletRotateHighNote[6]);
    if (notesB[a] < 3)dxl.setGoalPosition(7, malletRotateLowNote[7]); else dxl.setGoalPosition(7, malletRotateHighNote[7]);
    for (int i = 11; i < 18; i++) {
      dxl.setGoalPosition(i, malletUp);
    }
    delay(del);
  }
}

void playnotesB(int del) {
  startFrame = 32;
  endFrame = 63;

  for (int i = startFrame; i < endFrame + 1; i++) {
    // STRIKE NOTE
    if (notesC[i] == 1 || notesC[i] == 2)  dxl.setGoalPosition(11, malletDown);
    if (notesD[i] == 1 || notesD[i] == 2)  dxl.setGoalPosition(12, malletDown);
    if (notesE[i] == 1 || notesE[i] == 2)  dxl.setGoalPosition(13, malletDown);
    if (notesF[i] == 1 || notesF[i] == 2)  dxl.setGoalPosition(14, malletDown);
    if (notesG[i] == 1 || notesG[i] == 2)  dxl.setGoalPosition(15, malletDown);
    if (notesA[i] == 1 || notesA[i] == 2)  dxl.setGoalPosition(16, malletDown);
    if (notesB[i] == 1 || notesB[i] == 2)  dxl.setGoalPosition(17, malletDown);
    delay(time2Move);

    //PREP FOR NEXT NOTE
    if (i + 1 >= endFrame + 1) a = startFrame; else a = i + 1;
    if (notesC[a] == 1)dxl.setGoalPosition(1, malletRotateLowNote[1]); else dxl.setGoalPosition(1, malletRotateHighNote[1]);
    if (notesD[a] == 1)dxl.setGoalPosition(2, malletRotateLowNote[2]); else dxl.setGoalPosition(2, malletRotateHighNote[2]);
    if (notesE[a] == 1)dxl.setGoalPosition(3, malletRotateLowNote[3]); else dxl.setGoalPosition(3, malletRotateHighNote[3]);
    if (notesF[a] == 1)dxl.setGoalPosition(4, malletRotateLowNote[4]); else dxl.setGoalPosition(4, malletRotateHighNote[4]);
    if (notesG[a] == 1)dxl.setGoalPosition(5, malletRotateLowNote[5]); else dxl.setGoalPosition(5, malletRotateHighNote[5]);
    if (notesA[a] == 1)dxl.setGoalPosition(6, malletRotateLowNote[6]); else dxl.setGoalPosition(6, malletRotateHighNote[6]);
    if (notesB[a] == 1)dxl.setGoalPosition(7, malletRotateLowNote[7]); else dxl.setGoalPosition(7, malletRotateHighNote[7]);
    for (int i = 11; i < 18; i++) {
      dxl.setGoalPosition(i, malletUp);
    }
    delay(del);
  }
}

void playnotesC(int del) {
  startFrame = 64;
  endFrame = 160;

  for (int i = startFrame; i < endFrame + 1; i++) {
    // STRIKE NOTE
    if (notesC[i] == 1 || notesC[i] == 2)  dxl.setGoalPosition(11, malletDown);
    if (notesD[i] == 1 || notesD[i] == 2)  dxl.setGoalPosition(12, malletDown);
    if (notesE[i] == 1 || notesE[i] == 2)  dxl.setGoalPosition(13, malletDown);
    if (notesF[i] == 1 || notesF[i] == 2)  dxl.setGoalPosition(14, malletDown);
    if (notesG[i] == 1 || notesG[i] == 2)  dxl.setGoalPosition(15, malletDown);
    if (notesA[i] == 1 || notesA[i] == 2)  dxl.setGoalPosition(16, malletDown);
    if (notesB[i] == 1 || notesB[i] == 2)  dxl.setGoalPosition(17, malletDown);
    delay(time2Move);

    //PREP FOR NEXT NOTE
    if (i + 1 >= endFrame + 1) a = startFrame; else a = i + 1;
    if (notesC[a] == 1)dxl.setGoalPosition(1, malletRotateLowNote[1]); else dxl.setGoalPosition(1, malletRotateHighNote[1]);
    if (notesD[a] == 1)dxl.setGoalPosition(2, malletRotateLowNote[2]); else dxl.setGoalPosition(2, malletRotateHighNote[2]);
    if (notesE[a] == 1)dxl.setGoalPosition(3, malletRotateLowNote[3]); else dxl.setGoalPosition(3, malletRotateHighNote[3]);
    if (notesF[a] == 1)dxl.setGoalPosition(4, malletRotateLowNote[4]); else dxl.setGoalPosition(4, malletRotateHighNote[4]);
    if (notesG[a] == 1)dxl.setGoalPosition(5, malletRotateLowNote[5]); else dxl.setGoalPosition(5, malletRotateHighNote[5]);
    if (notesA[a] == 1)dxl.setGoalPosition(6, malletRotateLowNote[6]); else dxl.setGoalPosition(6, malletRotateHighNote[6]);
    if (notesB[a] == 1)dxl.setGoalPosition(7, malletRotateLowNote[7]); else dxl.setGoalPosition(7, malletRotateHighNote[7]);
    for (int i = 11; i < 18; i++) {
      dxl.setGoalPosition(i, malletUp);
    }
    delay(del);
  }
}
void lowerAllMallets() { //Slowly lowers all mallets to surface of drum
  for (int i = 1; i < 8; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, i + 10, 30);
    dxl.setGoalPosition(i + 10, malletDown);
    delay(10);
  }
  delay(1000);

  for (int i = 1; i < 8; i++) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, i + 10, 0);
    delay(10);
  }
}
void prepFirstNotes() {
  for (int i = 11; i < 18; i++) { //Mallets UP
    dxl.setGoalPosition(i, malletUp);
    delay(100);
  }
  //PREP FOR FIRST NOTE
  a = 0;
  if (notesC[a] == 1)dxl.setGoalPosition(1, malletRotateLowNote[1]); else dxl.setGoalPosition(1, malletRotateHighNote[1]);
  if (notesD[a] == 1)dxl.setGoalPosition(2, malletRotateLowNote[2]); else dxl.setGoalPosition(2, malletRotateHighNote[2]);
  if (notesE[a] == 1)dxl.setGoalPosition(3, malletRotateLowNote[3]); else dxl.setGoalPosition(3, malletRotateHighNote[3]);
  if (notesF[a] == 1)dxl.setGoalPosition(4, malletRotateLowNote[4]); else dxl.setGoalPosition(4, malletRotateHighNote[4]);
  if (notesG[a] == 1)dxl.setGoalPosition(5, malletRotateLowNote[5]); else dxl.setGoalPosition(5, malletRotateHighNote[5]);
  if (notesA[a] == 1)dxl.setGoalPosition(6, malletRotateLowNote[6]); else dxl.setGoalPosition(6, malletRotateHighNote[6]);
  if (notesB[a] == 1)dxl.setGoalPosition(7, malletRotateLowNote[7]); else dxl.setGoalPosition(7, malletRotateHighNote[7]);
  delay(500);
}

void torqueAllOn() { //turns all servos on
  for (int i = 1; i < 8; i++) {
    dxl.torqueOn(i);
    dxl.torqueOn(i + 10);
    delay(100);
  }
  delay(500);
}

void torqueAllOff() { //turns all servos off
  for (int i = 1; i < 8; i++) {
    dxl.torqueOff(i);
    dxl.torqueOff(i + 10);
    delay(100);
  }
  delay(500);
}

void startAnimation() {
  //All Mallets up and Center
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(i, 2024);
    dxl.setGoalPosition(i + 10, 1712);
  }
  delay(400);
  //All Mallets rotate side one at a time
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(reorder[i], 1730);
    delay(100);
  }
  delay(400);
  //All Mallets rotate center one at a time
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(reorder[i], 2024);
    delay(100);
  }
  delay(400);
  //All Mallets rotate side at once
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(reorder[i], 1730);
  }
  delay(700);
  //All Mallets rotate to start position
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(i, malletRotateLowNote[i]);
    dxl.setGoalPosition(i, 1712);
  }
  delay(700);

  //All Mallets rotate side at once
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(reorder[i], 1730);
  }
  delay(150);
  //All Mallets rotate to start position
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(i, malletRotateLowNote[i]);
    dxl.setGoalPosition(i, 1712);
  }
  delay(150);
  //All Mallets rotate side at once
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(reorder[i], 1730);
  }
  delay(150);
  //All Mallets rotate to start position
  for (int i = 1; i < 8; i++) {
    dxl.setGoalPosition(i, malletRotateLowNote[i]);
    dxl.setGoalPosition(i, 1712);
  }
  delay(1000);
  lowerAllMallets();//go to neutral Position and lower mallets slowly
  torqueAllOff();//Torques Servos Off
}

void loop() {
  buttonState = digitalRead(buttonPin); // check for button press
  if (buttonState == HIGH) { //if button is pressed then play song
    torqueAllOn(); //Torques Servos On
    prepFirstNotes();//Move to first note Start Position
    playnotesA(100); //plays sequence A at tempo of Millis between notes
    playnotesB(100);
    playnotesC(100);
    lowerAllMallets();//go to neutral Position and lower mallets slowly
    torqueAllOff();//Torques Servos Off
  }
}
