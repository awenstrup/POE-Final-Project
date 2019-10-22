//***********Include libraries***************
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//*************Define constants and global variables***************
#define throttlePin 0
#define steeringPin 0

#define rawThrottleMin 0
#define rawThrottleMax 1000

#define rawSteeringMin 0
#define rawSteeringMax 1000

unsigned long rawThrottle;
unsigned long rawSteering;

int throttleOut;
int steeringOut;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

//************Helper functions******************
void readThrottle() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void mapThrottle() { //Converts raw throttle data to output
  //For week 1, return in range 0 to 100
  throttleOut = rawThrottle;
}

void mapSteering() { //Converts raw steering data to output
  //For Week 1, return range -30 to 30
  //- values turn left, + values turn right
  steeringOut = rawSteering;
}

//************Debugging*****************8

//*************Setup and main loop**************
void setup() {
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);
  
  Serial.begin(9600);

  AFMS.begin();
  leftMotor->setSpeed(throttleOut);
  rightMotor->setSpeed(throttleOut);

  delay(1000);
}

void loop() {
  readThrottle();
  readSteering();
  char buff[64];
  sprintf(buff, "Throttle: %d Steering: %d", rawThrottle, rawSteering);
  Serial.println(buff);

  //leftMotor->setSpeed(throttleOut - steeringOut);
  //rightMotor->setSpeed(throttleOut + steeringOut);

  delay(500);
}
