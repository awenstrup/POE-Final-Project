//***********Include libraries***************
#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <NeoSWSerial.h>
#include <stdio.h>

//*************Define constants and global variables***************

//***********Define pins***************
//NOTE: Timer 2 is used for PWM on pins 3 and 11; do not fuck wit deez pinz
#define throttlePin 8 //PB0; PCINT0; column 1 on reciever side; left stick up/down
#define steeringPin 7 //PD6, PCINT22; column 0 on reciever; right stick left/right

#define steeringServoPin 5
#define frontMotorPin 10
#define rearMotorPin 6 //set to 11 before running

#define ssPin1 11
#define ssPin2 12

//****************Raw input values*********************
uint32_t rawThrottle = 0; //length of pulses
uint32_t rawSteering = 0;

//Min and max pulse lengths from reciever, used for mapping
#define rawThrottleMin 1000
#define rawThrottleMax 1970
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
#define throttleOutMin 1100
#define throttleOutMax 1900

//Min and max values to write to steering servo through motor shield
#define steeringCenter 90
#define steeringOutMax 35

const float steeringSensitivity = 0.8; //(0 means full steering at full throttle; 1 means no steering)

int throttleOut = 0; //output
int steeringOut = steeringCenter; //output

//***************Accelerometer Setup**************
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t a, m, g, temp;

//**************Other global definitions**************
volatile int timer = 0;
boolean go = false;

volatile boolean runAccelToBlue = false;

Servo steering;
Servo frontMotor;
Servo rearMotor;

//blue(ssPin1, ssPin2);

//************Setup*****************
void initTimer() { //addapted from https://www.instructables.com/id/Arduino-Timer-Interrupts/
  TCCR2A = 0;// set entire TCCR2A register to 0
  OCR2A = 128;// 64 hz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 1024 prescaler
  TCCR2B |= 0b00000111;   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
}

//*************Interupts*************
ISR(TIMER2_COMPA_vect) {
    timer++;
    if (timer == 1) {
      timer = 0;
      go = true;
      runAccelToBlue = true;
      //digitalWrite(13, !digitalRead(13));
    }
}

//************Helper functions******************
void readThrottle() { 
  rawThrottle = pulseIn(throttlePin, HIGH, 1000000); //Times out if no pulse detected in 1 second
}

void readSteering() {
  rawSteering = pulseIn(steeringPin, HIGH, 1000000); //Times out if no pulse detected in 1 second
}

void mapThrottle() { //Converts raw throttle data to 0-255
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
  /*Map according to steering
  int s = abs(128 - abs(128 - mappedSteering));
  float steeringScaler = (0.5 + s/255.0); //scales throttle from 0.5 to 1x full throttle based on steering angle
  */
  
  float scale = (mappedThrottle - mapLow)*1.0/(mapHigh-mapLow) * (throttleOutMax - throttleOutMin);
  throttleOut = throttleOutMin + (int)(scale);
}

void outputSteering() {
  /* throttleScaler is set to the square root of the distance to full speed, between 0 and 1 */
  double throttleScaler = (min(abs(mappedThrottle - mapHigh), abs(mappedThrottle - mapLow)) * (steeringSensitivity)) / ((mapHigh-mapLow)/2);
  throttleScaler = sqrt(throttleScaler);
  int steeringDelta = (throttleScaler + 1 - steeringSensitivity) * (steeringOutMax) * ((mappedSteering - (mapHigh/2))/(mapHigh/2.0)) ;

  steeringOut = (int) steeringCenter + steeringDelta;
}

void readAccel() {
  lsm.read();
  lsm.getEvent(&a, &m, &g, &temp);
}

void accelToBlue() {
  if (runAccelToBlue) {
    /*
    String out = "";
    runAccelToBlue = false;
    char buff[16];
    
    dtostrf(a.acceleration.x, 3, 2, buff);
    out = out + buff + ", ";
    dtostrf(a.acceleration.y, 3, 2, buff);
    out = out + buff + ", ";
    dtostrf(a.acceleration.z, 3, 2, buff);
    out = out + buff + ", ";
    
    dtostrf(g.gyro.x, 3, 2, buff);
    out = out + buff + ", ";
    dtostrf(g.gyro.y, 3, 2, buff);
    out = out + buff + ", ";
    dtostrf(g.gyro.z, 3, 2, buff);
    out = out + buff + ", ";
    
    ltoa(millis(), buff, 10);
    out = out + buff;
    Serial.println(out);
    */
    
    runAccelToBlue = false;
    char buff[32];

    ltoa(millis(), buff, 10);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(a.acceleration.x, 3, 2, buff);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(a.acceleration.z, 3, 2, buff);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(a.acceleration.y, 3, 2, buff);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(g.gyro.x, 3, 2, buff);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(g.gyro.y, 3, 2, buff);
    Serial.write(buff);
    Serial.write(", ");
    
    dtostrf(g.gyro.z, 3, 2, buff);
    Serial.write(buff);
    Serial.write("\n"); 
  }
}

void accelToSerial() {
  Serial.print("Acceleration (x, y, z)");
  Serial.print(")\n");
 
  char buff[32];

  Serial.print("(");
  dtostrf(a.acceleration.x, 3, 2, buff);
  Serial.print(buff);
  Serial.print(", ");
  dtostrf(a.acceleration.z, 3, 2, buff);
  Serial.print(buff);
  Serial.print(", ");
  dtostrf(a.acceleration.y, 3, 2, buff);
  Serial.print(buff);
  Serial.print(")\n");

  Serial.print("Rotational Acceleration (pitch, roll, yaw)");
  Serial.print(")\n");
    
  Serial.print("(");
  dtostrf(g.gyro.x, 3, 2, buff);
  Serial.print(buff);
  Serial.print(", ");
  dtostrf(g.gyro.y, 3, 2, buff);
  Serial.print(buff);
  Serial.print(", ");
  dtostrf(g.gyro.z, 3, 2, buff);
  Serial.print(buff);
  Serial.print(")\n");

  Serial.print("\n");
}

//************Debugging*****************
void printOuts() {
  char buff[32];
  sprintf(buff, "%d", throttleOut);
  Serial.println(buff);

  sprintf(buff, "%d", steeringOut);
  Serial.println(buff);

  Serial.println("");
}

//*************Setup and main loop**************
void setup() {
  //Set pins to receiver as input
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);

  //Set pin 13 as debugging LED
  //pinMode(13, OUTPUT);

  //Open serial port
  Serial.begin(9600); //to and from accelerometer
  //blue.begin(9600); //to bluetooth module
  lsm.begin();

  steering.attach(steeringServoPin);
  frontMotor.attach(frontMotorPin);
  rearMotor.attach(rearMotorPin);

  initTimer();
  sei();
}

void loop() {
  if (true) {
    //go = false;
    
    //Read raw throttle and steering values from receiver
    //readThrottle();
    //readSteering();
    
    //Map raw throttle and steering data
    mapThrottle();
    mapSteering();

    //Remap throttle and steering data to output values
    outputThrottle();
    outputSteering();
    
    //Drive rear wheels
    frontMotor.writeMicroseconds(throttleOut);
    rearMotor.writeMicroseconds(throttleOut);

    //Update steering angle
    steering.write(steeringOut);

    //Read accelerometer data
    readAccel();
    
    //Print acceleromter data
    accelToBlue();
    
    //printOuts();
  }
}
