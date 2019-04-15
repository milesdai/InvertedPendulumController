#define POT_PIN A0
#define DIR_PIN 6
#define STEP_PIN 3

#define STEPS_PER_REV 200
#define MIN_PULSE_DELAY 1400
#define CENTER_POT_VAL 755 // 90 degrees occurs at 520

#define MASS 10
#define dT 10 // in milliseconds
#define Kp 2
#define Kd 0.1

int steps[3] = {0, 0, 0};
int pastError = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(13, OUTPUT);

  Serial.begin(19200);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  int error = potValue - CENTER_POT_VAL;

  int proportional = error * Kp;
  float derivative = (error - pastError) / ((float)dT) * Kd; // division after scaling

  
  
  int desiredForce = -1*(proportional + derivative);
  steps[2] = getSteps(desiredForce);
  step(steps[2]);

  Serial.print("*************\nProportional: ");
  Serial.println(proportional);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.print("Steps: [");
  Serial.print(steps[0]);
  Serial.print(", ");
  Serial.print(steps[1]);
  Serial.print(", ");
  Serial.print(steps[2]);
  Serial.println("]\n****************");

  // Shift steps left by 1
  steps[0] = steps[1];
  steps[1] = steps[2];

  pastError = error;
  digitalWrite(13, !digitalRead(13));
  delay(dT);
}

int getSteps(int desiredForce) {
  int accel = desiredForce / MASS;
  return ((steps[1]) / dT + accel) * dT;
}

void testStepper() {
  for(int i = 1; i < 11; i++) {
    step(STEPS_PER_REV / 10);
  }
  for(int i = 10; i > 0; i--) {
    step(-1 * STEPS_PER_REV / 10);
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
  numSteps = min(numSteps, 50);
  for(int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1400);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1400);
  }
}

