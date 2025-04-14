#include <Servo.h>

#define BUTTON_PIN 2 // Button connected to pin 2
#define CLAMP_PIN1 6 // Right clamp servo w/ charity updates
#define CLAMP_PIN2 3 // Left clamp servo w/ charity updates
#define CLAMP_PIN3 5 // Page flipper w/ charity updates

const int wheelServoIn1 = 7;
const int wheelServoIn2 = 8;
const int enablePin = 4;

// ranger variables
const int trigPin = 14;
const int echoPin = 15;
float duration, distance;

// servo initiation
Servo clampServo1;
Servo clampServo2;
Servo flipServo;

// variables for the deadzone of the flipper
int flipStart = 150; // initial position
int flipEnd = 0;     // target position
int flipCurrent = 150;
bool flipping = false;
unsigned long lastFlipUpdate = 0;
const unsigned long flipInterval = 15; // ms between position updates
const int deadzone = 4;                // Tolerance to stop jitter at target
const int maxFlipSpeed = 10;            // Max degrees moved per update
const int minFlipSpeed = 1;            // Minimum movement when near target
const int flipThreshold = 15;          // Threshold for switching to minFlipSpeed


// SETUP
void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // assigning the clamps to their servo variables and setting their initial positions
  clampServo1.attach(CLAMP_PIN1);
  clampServo1.write(80);
  clampServo2.attach(CLAMP_PIN2);
  clampServo2.write(0);
  flipServo.attach(CLAMP_PIN3);
  flipServo.write(flipCurrent);

  // assigning the wheel to its pins and stuff
  pinMode(enablePin, OUTPUT);
  pinMode(wheelServoIn1, OUTPUT);
  pinMode(wheelServoIn2, OUTPUT);

  //  for the ranger i think ?
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {

  // ranger readingx
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;


  if (digitalRead(BUTTON_PIN) == HIGH && !flipping ) {
    Serial.println("Button pressed");

    // Lift clamps
    clampServo1.write(0);
    clampServo2.write(100);
    delay(350);

    // Spin motor
    digitalWrite(wheelServoIn1, HIGH);
    digitalWrite(wheelServoIn2, LOW);
    analogWrite(enablePin, 255);
    delay(1000);
    digitalWrite(wheelServoIn1, LOW);
    digitalWrite(wheelServoIn2, LOW);
    analogWrite(enablePin, 0);

    // Start slow flip
    flipping = true;
    flipStart = flipCurrent;
    flipEnd = 30; // move to 0
    lastFlipUpdate = millis();
  }

  // Handle slow flipping
  if (flipping && millis() - lastFlipUpdate >= flipInterval) {
    lastFlipUpdate = millis();

    // Check if we are close to the target (deadzone tolerance)
    if (abs(flipCurrent - flipEnd) <= deadzone) {
      flipCurrent = flipEnd;
      flipServo.write(flipCurrent);

      if (flipEnd == 0) {
        delay(500);        // small pause while flipped
        flipEnd = 190;     // go back up
      } else if (flipEnd == 190) {
        flipping = false;

        // Clamp pages down after flipping is complete
        clampServo1.write(80);
        clampServo2.write(0);
        delay(1000);
      }

    } else {
      // Gradually adjust speed based on proximity to target
      int distanceToTarget = abs(flipCurrent - flipEnd);
      int flipSpeed = maxFlipSpeed;

      if (distanceToTarget < flipThreshold) {
        flipSpeed = minFlipSpeed;  // Slow down near the target position
      }

      if (flipCurrent > flipEnd) {
        flipCurrent -= flipSpeed;
        if (flipCurrent < flipEnd) flipCurrent = flipEnd;
      } else {
        flipCurrent += flipSpeed;
        if (flipCurrent > flipEnd) flipCurrent = flipEnd;
      }
      flipServo.write(flipCurrent);
    }
  }
}
