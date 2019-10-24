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
#define throttleOutMax 255

#define steeringOutMin 50
#define steeringOutMax 110

unsigned long rawThrottle;
unsigned long rawSteering;

int throttleOut;
int steeringOut;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *drivingMotor1 = AFMS.getMotor(1); //motor 1 on shield
Adafruit_DCMotor *drivingMotor2 = AFMS.getMotor(2); //motor 2 on shield


Servo steering;
#define steeringServoPin 9

//************Helper functions******************
void readThrottle() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  rawSteering = pulseIn(steeringPin, HIGH);
}

void mapThrottle() { //Converts raw throttle data to output
  //For week 1, return in range 0 to 100
  if (rawThrottle < (rawThrottleMin + rawThrottleDeadZone)) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;

  throttleOut = (rawThrottle - rawThrottleMin) * (throttleOutMax - throttleOutMin) / (rawThrottleMax - rawThrottleMin);
}

void mapSteering() { //Converts raw steering data to output
  //For week 1, return between 60 and 120
  if (rawSteering < rawSteeringMin) rawSteering = rawSteeringMin;
  else if (rawSteering > rawSteeringMax) rawSteering = rawSteeringMax;

  steeringOut = steeringOutMin + (rawSteering - rawSteeringMin) * (steeringOutMax - steeringOutMin) / (rawSteeringMax - rawSteeringMin);
  //steeringOut = 80;
}

//************Debugging*****************8

//*************Setup and main loop**************
void setup() {
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);
  
  Serial.begin(9600);

  AFMS.begin();
  drivingMotor1->setSpeed(0);
  drivingMotor2->setSpeed(0);

  steering.attach(9);

  delay(500);
}

void loop() {
  readThrottle();
  readSteering();

  mapThrottle();
  mapSteering();
  
  //char buff[32];
  //sprintf(buff, "Steering: %d", steeringOut);
  //Serial.println(buff);

  drivingMotor1->setSpeed(throttleOut);
  drivingMotor1->run(BACKWARD);
  
  drivingMotor2->setSpeed(throttleOut);
  drivingMotor2->run(FORWARD);
  
  steering.write(steeringOut);

  //delay(200);
}
