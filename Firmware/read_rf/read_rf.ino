#define throttlePin 0
#define steeringPin 0

#define rawThrottleMin 0
#define rawThrottleMax 1000

#define rawSteeringMin 0
#define rawSteeringMax 1000

unsigned long rawThrottle;
unsigned long rawSteering;

void setup() {
  pinMode(throttlePin, INPUT);
  pinMode(steeringPin, INPUT);
  Serial.begin(9600);

}

void loop() {
  readThrottle();
  readSteering();
  String out = "Throttle: " + rawThrottle + " Steering: " + rawSteering;
  Serial.println(out);

}

void readThrottle() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}

void readSteering() {
  rawThrottle = pulseIn(throttlePin, HIGH);
}
