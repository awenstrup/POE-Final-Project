//***********Include libraries***************
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

//*************Define constants and global variables***************
#define throttlePin 5 //column 2 on reciever side; left stick up/down
#define steeringPin 6 //column 0 on reciever; right stick left/right

//Min and max pulse lengths from reciever, used for mapping
#define rawThrottleMin 1054
#define rawThrottleMax 1888
#define rawThrottleDeadZone 30

#define rawSteeringMin 1052
#define rawSteeringMax 1886

//Min and max values to write to DC motors through motor shield
#define throttleOutMin 0
#define throttleOutMax 255

//Min and max values to write to steering servo through motor shield
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
  //rudimentary implimentation used pulseIn arduino function
  //in future, either feed PWM though RC filter and analogRead, or use interupts
  //to read PWM more efficiently
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  //rudimentary implimentation used pulseIn arduino function
  //in future, either feed PWM though RC filter and analogRead, or use interupts
  //to read PWM more efficiently
  rawSteering = pulseIn(steeringPin, HIGH);
}

void mapThrottle() { //Converts raw throttle data to output
  //For week 1, return in range 0 to 255
  if (rawThrottle < (rawThrottleMin + rawThrottleDeadZone)) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;

  throttleOut = (rawThrottle - rawThrottleMin) * (throttleOutMax - throttleOutMin) / (rawThrottleMax - rawThrottleMin);
}

void mapSteering() { //Converts raw steering data to output
  //For week 1, return between 50 and 110
  if (rawSteering < rawSteeringMin) rawSteering = rawSteeringMin;
  else if (rawSteering > rawSteeringMax) rawSteering = rawSteeringMax;

  steeringOut = steeringOutMin + (rawSteering - rawSteeringMin) * (steeringOutMax - steeringOutMin) / (rawSteeringMax - rawSteeringMin);
}

//************Debugging*****************
void printSteering() {
  char buff[32];
  sprintf(buff, "Steering: %d", steeringOut);
  Serial.println(buff);
}

//*************Setup and main loop**************
void setup() {
  //Set pins to receiver as input
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  //Open serial port
  Serial.begin(9600);

  AFMS.begin();
  drivingMotor1->setSpeed(0);
  drivingMotor2->setSpeed(0);

  steering.attach(9);

  delay(500);
}

void loop() {
  //Read raw throttle and steering values from receiver
  readThrottle();
  readSteering();

  //Map raw throttle and steering data to output values
  mapThrottle();
  mapSteering();

  //Drive rear wheels
  drivingMotor1->setSpeed(throttleOut);
  drivingMotor1->run(BACKWARD);
  
  drivingMotor2->setSpeed(throttleOut);
  drivingMotor2->run(FORWARD);

  //Update steering angle
  steering.write(steeringOut);
}
