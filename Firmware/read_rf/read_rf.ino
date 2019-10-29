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
#define rawThrottleDeadZone 80

#define rawSteeringMin 1052
#define rawSteeringMax 1886

//Min and max values to write to DC motors through motor shield
#define throttleOutMin -255
#define throttleOutMax 255

//Min and max values to write to steering servo through motor shield
#define steeringOutMin 50
#define steeringOutMax 110

unsigned long timer;

unsigned long rawThrottle;
unsigned long rawSteering;

boolean throtPinState = false;
boolean steerPinState = false;

int throttleOut;
int steeringOut;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *drivingMotor1 = AFMS.getMotor(1); //motor 1 on shield
Adafruit_DCMotor *drivingMotor2 = AFMS.getMotor(2); //motor 2 on shield


Servo steering;
#define steeringServoPin 9

//************Setup*****************
void init_timer() { //addapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 1Mhz increments
  OCR2A = 1;// matches every other time
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void init_pcint() {
  //Enables interupts on shutdown sense pins
  PCICR |= 0b00000100; //enables PCINTs 16-23 
  PCMSK0 |= 0b01100000; //enables PCINT21 and PCINT22
}


//*************Interupts*************
ISR(TIMER0_COMPA_vect) {
    //count 1 million times/second
    timer++;
}

ISR(PCINT2_vect) {
    //TODO
}



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
  int rawThrottleMidpoint = (rawThrottleMax + rawThrottleMin) / 2;

  if (abs(rawThrottle - rawThrottleMidpoint) < rawThrottleDeadZone) rawThrottle = rawThrottleMidpoint;
  else if (rawThrottle < rawThrottleMin) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;

  throttleOut = throttleOutMin + (rawThrottle - rawThrottleMin) * (throttleOutMax - throttleOutMin) / (rawThrottleMax - rawThrottleMin);
}

void mapSteering() { //Converts raw steering data to output
  //For week 1, return between 50 and 110
  if (rawSteering < rawSteeringMin) rawSteering = rawSteeringMin;
  else if (rawSteering > rawSteeringMax) rawSteering = rawSteeringMax;

  steeringOut = steeringOutMin + (rawSteering - rawSteeringMin) * (steeringOutMax - steeringOutMin) / (rawSteeringMax - rawSteeringMin);
}

//************Debugging*****************
void printThrottle() {
  char buff[32];
  sprintf(buff, "Raw Throttle: %d", rawThrottle);
  Serial.println(buff);

  sprintf(buff, "Throttle Out: %d", throttleOut);
  Serial.println(buff);

  delay(400);
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
  drivingMotor1->setSpeed(throttleOut>0 ? throttleOut : -throttleOut);
  drivingMotor1->run(throttleOut>0 ? BACKWARD : FORWARD);
  
  drivingMotor2->setSpeed(throttleOut>0 ? throttleOut : -throttleOut);
  drivingMotor2->run(throttleOut>0 ? FORWARD : BACKWARD);

  //printThrottle();

  //Update steering angle
  steering.write(steeringOut);
}
