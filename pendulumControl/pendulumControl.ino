#define POT_PIN A0
#define DIR_PIN 6
#define STEP_PIN 3

#define STEPS_PER_REV 200
#define MIN_PULSE_DELAY 1400

//#include<Stepper.h>

//Stepper stepper(STEPS_PER_REV, 8, 9, 10, 11);

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(POT_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
//  testStepper();
  int pot = analogRead(POT_PIN);
  Serial.println(pot);
  delay(10);
}

int readPot() {
  return analogRead(POT_PIN);
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
  for(int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1400);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1400);
  }
}

