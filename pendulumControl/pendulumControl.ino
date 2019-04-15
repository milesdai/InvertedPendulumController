#define POT_PIN A0
#define DIR_PIN 6
#define STEP_PIN 3

#define STEPS_PER_REV 200
#define MIN_PULSE_DELAY 1400
#define CENTER_POT_VAL 754.9 // 90 degrees occurs at 520 ; higher  to move setpoint away

#define MASS 10
#define NUM_STEPS 2
#define Kp 200
#define Kd 400

int pastError = 0;
int pastDT = 10;

bool debug = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(13, OUTPUT);

  if(debug){
    Serial.begin(19200);  
  }
  Serial.begin(19200);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  float error = potValue - CENTER_POT_VAL;
  error =  error;
  //Serial.println(error);

//  int steps = (int)(error * -1);
//  Serial.println(steps);
//  step(steps);

  float proportional = error * Kp;
  float derivative = (error - pastError) / ((float)pastDT + 0.001) * Kd; // division after scaling

  int desiredForce = -1*(proportional + derivative);
  double currentDT = getDT(desiredForce);
  stepDT(currentDT);

  if(debug){
    //Serial.print("*************\nProportional: ");
    //Serial.println(proportional);
    //Serial.print("Derivative: ");
    //Serial.println(derivative);
    //Serial.println("\n****************");  
    //Serial.println(desiredForce);
  }

  pastDT = abs(currentDT);
  pastError = error;
  
//  digitalWrite(13, !digitalRead(13));
}

double getDT(int desiredForce) {
  bool neg = desiredForce < 0;
  double acc = abs(desiredForce) / (double)MASS;
  double dT = -1 * NUM_STEPS - sqrt(4 * acc * NUM_STEPS * pastDT * pastDT + NUM_STEPS * NUM_STEPS) / 2 / acc / (0.001+ pastDT);
  dT = 2000 / (acc + 5);
  return neg ? -1 * dT : dT;
}

void testStepper() {
  for(int i = 1; i < 11; i++) {
    step(STEPS_PER_REV / 10);
  }
  for(int i = 10; i > 0; i--) {
    step(-1 * STEPS_PER_REV / 10);
  }
}

void stepDT(double dT) {
  digitalWrite(DIR_PIN, LOW);
  if(dT < 0) {
    digitalWrite(DIR_PIN, HIGH);
    dT = -1 * dT;
  }
  if(debug) {
    ////Serial.print("dT: ");
    //Serial.println(dT);
  }
  //int delayTime = (dT * 1000) / NUM_STEPS / 2;
  int delayTime = dT;
  delayTime = max(delayTime, 30);
  delayTime = min(delayTime, 50);
  delayTime = map(delayTime, 30, 50, 1400, 3000);
  //Serial.println(delayTime);
  for(int i = 0; i < NUM_STEPS; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayTime);
  }
}

void step(int numSteps) {
  if(numSteps < 0){
    digitalWrite(DIR_PIN, HIGH);
    numSteps = -1 * numSteps;
  } else {
    digitalWrite(DIR_PIN, LOW);
  }
  Serial.print("Num steps (unclipped): ");
  Serial.println(numSteps);
  numSteps = min(numSteps, 100);
  
  for(int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1400);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1400);
  }
}

