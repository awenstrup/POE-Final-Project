//***********Include libraries***************
#include <Servo.h>

//*************Define constants and global variables***************
#define throttlePin 8 //PB0; PCINT0; column 1 on reciever side; left stick up/down
#define steeringPin 6 //PD6, PCINT22; column 0 on reciever; right stick left/right

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

unsigned long timer = 0;
unsigned long lastThrotInt = 0;
unsigned long lastSteerInt = 0;
boolean go = false;

unsigned long rawThrottle = 0;
unsigned long rawSteering = 0;

//Stores the current state of the throttle and steering pins
boolean throtPinState = false;
boolean steerPinState = false;

//Set to true when a pin's state is changed
boolean throttleUpdate = false;
boolean steeringUpdate = false;

//Stores the time at which the pin went high
unsigned long steerStartTime = 0;
unsigned long throtStartTime = 0;

int throttleOut = 0;
int steeringOut = 80;

Servo steering;
Servo frontMotor;
Servo rearMotor;
#define steeringServoPin 9
#define frontMotorPin 10
#define rearMotorPin 3 //set to 11 before running

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

void initPCINT() {
  //Enables interupts on shutdown sense pins
  PCICR |= 0b00000101; //enables PCINTs 16-23, 0-7
  PCMSK0 |= 0b00000001; //enables PCINT0
  PCMSK2 |= 0b01000000; //enables PCINT22
}


//*************Interupts*************
ISR(TIMER2_COMPA_vect) {
    timer++;
    if (true) go = true;
}

ISR(PCINT0_vect) {
  if (micros() - lastThrotInt > 300) {
    lastThrotInt = micros();
    boolean t1 = digitalRead(throttlePin);
    if (throttleUpdate) {
      if (t1) throtStartTime = micros();
      else rawThrottle = micros() - throtStartTime;
  
      throttleUpdate = false;
    }
    //digitalWrite(13, !digitalRead(13));
  }
}

ISR(PCINT2_vect) {
  boolean t1 = digitalRead(steeringPin);
  steeringUpdate = true;
  steerPinState = t1;
  steerStartTime = timer;
  //digitalWrite(13, !digitalRead(13));
}



//************Helper functions******************
void readThrottle() { 
  //rudimentary implimentation used pulseIn arduino function
  //in future, either feed PWM though RC filter and analogRead, or use interupts
  //to read PWM more efficiently
  rawThrottle = pulseIn(throttlePin, HIGH);

  //throttleUpdate = true;
}

void readSteering() {
  //rudimentary implimentation used pulseIn arduino function
  //in future, either feed PWM though RC filter and analogRead, or use interupts
  //to read PWM more efficiently
  rawSteering = pulseIn(steeringPin, HIGH);

  /*
  if (steeringUpdate) {
    if (steerPinState) {
      steerStartTime = timer;
    }
    else {
      rawSteering = timer - steerStartTime;
    }
    steeringUpdate = false;
  }
  */
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
  char buff[64];
  sprintf(buff, "Raw throttle: %d", rawThrottle);
  Serial.println(buff);
}

//*************Setup and main loop**************
void setup() {
  //Set pins to receiver as input
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  //Set pin 13 as debugging LED
  pinMode(9, OUTPUT);
  //pinMode(13, OUTPUT);

  //Open serial port
  Serial.begin(9600);

  steering.attach(steeringPin);
  frontMotor.attach(frontMotorPin);
  rearMotor.attach(rearMotorPin);

  initTimer();
  initPCINT();
  sei();
}

void loop() {
  if (go) {
    go = false;
    //Read raw throttle and steering values from receiver
    readThrottle();
    //readSteering();

    //Map raw throttle and steering data to output values
    //mapThrottle();
    //mapSteering();
  
    //Drive rear wheels
    frontMotor.write(throttleOut);
    rearMotor.write(throttleOut);

    //Update steering angle
    steering.write(steeringOut);

    printThrottle();
  }
}
