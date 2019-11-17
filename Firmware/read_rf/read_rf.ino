//***********Include libraries***************
#include <Servo.h>

//*************Define constants and global variables***************

//***********Define pins***************
#define throttlePin 8 //PB0; PCINT0; column 1 on reciever side; left stick up/down
#define steeringPin 6 //PD6, PCINT22; column 0 on reciever; right stick left/right

#define steeringServoPin 9
#define frontMotorPin 10
#define rearMotorPin 3 //set to 11 before running

//****************Raw input values*********************
uint32_t rawThrottle = 0; //length of pulses
uint32_t rawSteering = 0;

//Min and max pulse lengths from reciever, used for mapping
#define rawThrottleMin 1000
#define rawThrottleMax 1988
#define rawThrottleDeadZone 80

#define rawSteeringMin 994
#define rawSteeringMax 1987

//******************Mapping values**************
#define mapLow 0
#define mapHigh 255

int mappedThrottle = 0; //always maps 0-255
int mappedSteering = 128; //always maps 0-255

//**************Output values**********************************
//Min and max values to write to DC motors through motor shield
#define throttleOutMin 0
#define throttleOutMax 255

//Min and max values to write to steering servo through motor shield
#define steeringOutMin 50
#define steeringOutMax 110

int throttleOut = 0; //output
int steeringOut = 80; //output

//**************Other global definitions**************
volatile int timer = 0;
boolean go = false;

Servo steering;
Servo frontMotor;
Servo rearMotor;

//************Setup*****************
void initTimer() { //addapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
  TCCR2A = 0;// set entire TCCR2A register to 0
  OCR2A = 244;// 64 hz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= 0b00000111;   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

//*************Interupts*************
ISR(TIMER2_COMPA_vect) {
    timer++;
    if (timer == 10) {
      timer = 0;
      go = true;
      //digitalWrite(13, !digitalRead(13));
    }
}

//************Helper functions******************
void readThrottle() { 
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  rawSteering = pulseIn(steeringPin, HIGH);
}

void mapThrottle() { //Converts raw throttle data to 0-255
  /*With reverse
  int rawThrottleMidpoint = (rawThrottleMax + rawThrottleMin) / 2;
  if (abs(rawThrottle - rawThrottleMidpoint) < rawThrottleDeadZone) rawThrottle = rawThrottleMidpoint;
  else if (rawThrottle < rawThrottleMin) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;
  */

  /*Without reverse*/
  if (rawThrottle < rawThrottleMin + rawThrottleDeadZone) rawThrottle = rawThrottleMin;
  else if (rawThrottle > rawThrottleMax) rawThrottle = rawThrottleMax;
  mappedThrottle = mapLow + (rawThrottle - rawThrottleMin) * (mapHigh - mapLow) / (rawThrottleMax - rawThrottleMin);
}

void mapSteering() { //Converts raw steering data to 0-255
  if (rawSteering < rawSteeringMin) rawSteering = rawSteeringMin;
  else if (rawSteering > rawSteeringMax) rawSteering = rawSteeringMax;

  mappedSteering = mapLow + (rawSteering - rawSteeringMin) * (mapHigh - mapLow) / (rawSteeringMax - rawSteeringMin);
}

void outputThrottle() {
  int s = abs(128 - abs(128 - mappedSteering));
  float steeringScaler = (0.5 + s/255.0); //scales throttle from 0.5 to 1x full throttle based on steering angle
  throttleOut = (int) (mappedThrottle * steeringScaler);
}

//************Debugging*****************
void printThrottle() {
  char buff[64];
  sprintf(buff, "Raw throttle: %d", throttleOut);
  Serial.println(buff);
}

//*************Setup and main loop**************
void setup() {
  //Set pins to receiver as input
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  //Set pin 13 as debugging LED
  //pinMode(13, OUTPUT);

  //Open serial port
  Serial.begin(9600);

  steering.attach(steeringServoPin);
  frontMotor.attach(frontMotorPin);
  rearMotor.attach(rearMotorPin);

  initTimer();
  sei();
}

void loop() {
  if (go) {
    go = false;
    //Read raw throttle and steering values from receiver
    readThrottle();
    readSteering();

    //Map raw throttle and steering data
    mapThrottle();
    mapSteering();

    //Remap throttle and steering data to output values
    outputThrottle();
  
    //Drive rear wheels
    //frontMotor.write(throttleOut);
    //rearMotor.write(throttleOut);

    //Update steering angle
    //steering.write(steeringOut);

    printThrottle();
  }
}
