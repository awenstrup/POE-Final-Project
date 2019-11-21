//      ******************************************************************
//      *                                                                *
//      *             ESC - Motor Speed Control Test Sketch              *
//      *                                                                *
//      ******************************************************************

#include <Servo.h>



//
// create the servo objects
//
Servo ESCServo;


//
// miscellaneous pin assignments
//
const int LED_PIN = 13;
const int ESC_SERVO_PIN = 10;

//
// global vars
//
float currentSpeed = 0.0;


// ---------------------------------------------------------------------------------
//                              Hardware and software setup
// ---------------------------------------------------------------------------------

//
// top level setup function
//
void setup()
{ 
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT); 

  //
  // initialize the ESC servo connection
  //
  ESCServo.attach(ESC_SERVO_PIN);
  setESCSpeedValue(0.0);
}



//
// main loop
//
void loop()
{
 calibrateThrottleRange();
 testThrottleRange();
}



//
// calibrate the throttle range
//
void calibrateThrottleRange()
{
  //
  // set throttle to neutral position
  //
  setESCSpeedValue(0.0);

  //
  // give the user instructions
  //
  Serial.println("Hold the ESC button.");
  Serial.println("Turn on ESC.");
  Serial.println("Quickly press.");
  Serial.println("Then press Enter.");
  
 
  //
  // wait for a press of the Enter button
  //
  while(Serial.read() == -1)
    ;
  
  //
  // set full throttle
  //
  setESCSpeedValue(1.0);
  Serial.println("Now at full forward.");
  delay(4000);


  //
  // set full reverse
  //
  setESCSpeedValue(-1.0);
  Serial.println("Now at full reverse.");
  delay(2000);


  //
  // give the user instructions
  //
  Serial.println("Release the ESC button, then press Enter.");
  
 
  //
  // wait for a press of the Enter button
  //
  while(Serial.read() == -1)
    ;

  //
  // set speed back to 0
  //
  currentSpeed = 0.0;
  setESCSpeedValue(0.0);

  Serial.println("");
  Serial.println("");
  Serial.println("");
}





//
// test throttle range
//
void testThrottleRange()
{
  //
  // give the user instructions
  //
  Serial.println("Press Enter to go Full Forward");  
 
  //
  // wait for a press of the Enter button
  //
  while(Serial.read() == -1)
    ;
  
  //
  // set full throttle
  //
  setESCSpeedValue(1.0);
  Serial.println("Now at full forward.");
  delay(3000);
  setESCSpeedValue(0.0);


  //
  // give the user instructions
  //
  Serial.println("Press Enter to go Full Reverse");  
 
  //
  // wait for a press of the Enter button
  //
  while(Serial.read() == -1)
    ;
  
  //
  // set full throttle
  //
  setESCSpeedValue(-1.0);
  Serial.println("Now at full reverse.");
  delay(3000);
  setESCSpeedValue(0.0);

  Serial.println("");
  Serial.println("");
  Serial.println("");
}



// ---------------------------------------------------------------------------------
//                                   Servo functions
// ---------------------------------------------------------------------------------

//
// set the ESC speed, note: calling this function takes 5us
//  Enter:  speed = speed scaler (-1.0 = full reverse, 0.0 = stop, 1.0 = full forward
//
void setESCSpeedValue(float speed)
{
  const int fullReverseUS = 1100;
  const int fullForwardUS = 1900;
  const int centerUS = (fullForwardUS + fullReverseUS) / 2;
  const float halfRangeUS = (float)(fullForwardUS - centerUS);

  if (speed < -1.0) speed = -1.0;
  if (speed > 1.0) speed = 1.0;

  int pwmTime = centerUS + (int) round(speed * halfRangeUS);
  ESCServo.writeMicroseconds(pwmTime);
}


// -------------------------------------- End --------------------------------------
