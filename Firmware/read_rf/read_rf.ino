//***********Include libraries***************
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

//*************Define constants and global variables***************
#define throttlePin 5 //column 2 on reciever side; left stick up/down
#define steeringPin 6 //column 0 on reciever; right stick left/right

#define rawThrottleMin 1054
#define rawThrottleMax 1888
#define rawThrottleDeadZone 30

#define rawSteeringMin 1052
#define rawSteeringMax 1886

#define throttleOutMin 0
#define throttleOutMax 100

#define steeringOutMin 0
#define steeringOutMax 0

unsigned long rawThrottle;
unsigned long rawSteering;

int throttleOut;
int steeringOut;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *drivingMotor = AFMS.getMotor(1);

Servo steering;
#define steeringServoPin 9

//************Helper functions******************
void readThrottle() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void mapThrottle() { //Converts raw throttle data to output
  //For week 1, return in range 0 to 100
  if (rawThrottle < (rawThrottleMin + rawThrottleDeadZone)) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;

  throttleOut = (rawThrottle - rawThrottleMin) * (throttleOutMax - throttleOutMin) / (rawThrottleMax - rawThrottleMin);
}

void mapSteering() { //Converts raw steering data to output
  //For Week 1, return range -50 to 50
  //- values turn left, + values turn right
  steeringOut = 0;
}

//************Debugging*****************8

//*************Setup and main loop**************
void setup() {
  pinMode(throttlePin, INPUT);
  
  Serial.begin(9600);

  AFMS.begin();
  drivingMotor->setSpeed(throttleOut);

  steering.attach(steeringServoPin);

  delay(1000);
}

void loop() {
  readThrottle();
  readSteering();
  char buff[32];
  sprintf(buff, "Pin reading: %d", rawThrottle);
  Serial.println(buff);

  drivingMotor->setSpeed(throttleOut);
  //steering.write(steeringOut);
  
  delay(500);
}
