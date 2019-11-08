//***********Include libraries***************
#include <Servo.h>

//*************Define constants and global variables***************
#define throttlePin 8 //PB0; PCINT0; column 1 on reciever side; left stick up/down
#define steeringPin 6 //PD6; PCINT22; column 0 on reciever; right stick left/right
#define esc1 10 //ESC PWM coming from pin 7

//Min and max pulse lengths from reciever, used for mapping
#define rawThrottleMin 1054
#define rawThrottleMax 1888
#define rawThrottleDeadZone 30

#define rawSteeringMin 1052
#define rawSteeringMax 1886

//Min and max values to write to DC motors through motor shield
#define throttleOutMin 10
#define throttleOutMax 100

//Min and max values to write to steering servo through motor shield
#define steeringOutMin 50
#define steeringOutMax 110

unsigned long rawThrottle;
unsigned long rawSteering;

int throttleOut;
int steeringOut;

Servo steering;
#define steeringServoPin 9

//************Setup*****************
void initTimer() { //addapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
  TCCR2A = 0;// set entire TCCR2A register to 0
  OCR2A = 255;// matches every 255 times
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= 0b00000111;
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void initPCINT() {
  //Enables interupts on shutdown sense pins
  PCICR |= 0b00000100; //enables PCINTs 16-23, 0-7
  PCMSK0 |= 0b00000001; //enables PCINT0
  PCMSK2 |= 0b01000000; //enables PCINT22
}


//*************Interupts*************
ISR(TIMER2_COMPA_vect) {
    //count 800,000 times/second
    timer++;
    if (timer % 20 == 0) go = true;
}

ISR(PCINT0_vect) {
  
}

ISR(PCINT2_vect) {
  
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
  if (rawThrottle < (rawThrottleMin + rawThrottleDeadZone)) rawThrottle = rawThrottleMin;
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
  sprintf(buff, "Throttle (raw): %d", rawThrottle);
  Serial.println(buff);

  sprintf(buff, "Throttle (out): %d", throttleOut);
  Serial.println(buff);
  delay(300);
}

//*************Setup and main loop**************
void setup() {
  //Set pins to receiver as input
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  //Set ESC pins as output
  pinMode(esc1, OUTPUT);

  //Open serial port
  Serial.begin(9600);

  //Attach steering servo
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
  analogWrite(esc1, throttleOut);

  //Update steering angle
  steering.write(steeringOut);

  //printThrottle();
}
