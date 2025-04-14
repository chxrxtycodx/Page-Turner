#include <Servo.h>

#define BUTTON_PIN 2         // Button connected to pin 2
#define CLAMP_PIN1 5         // Right clamp servo
#define CLAMP_PIN2 8         // Left clamp servo
#define CLAMP_PIN3 3         // Page flipper servo
#define LED_BUILTIN 13       // Built-in LED

const int wheelServoIn1 = 6;
const int wheelServoIn2 = 7;
const int enablePin = 4;

// Ranger pins (currently unused)
const int trigPin = 14;
const int echoPin = 15;

// Servo objects
Servo clampServo1;
Servo clampServo2;
Servo flipServo;

// Right clamp threshold
const int clamp1Deadzone = 3;
int lastClamp1Pos = -1;

// Flipper motion
int flipStart = 170;
int flipEnd = 0;
int flipCurrent = 170;
bool flipping = false;
unsigned long lastFlipUpdate = 0;
const unsigned long flipInterval = 15; // smoother movement
const int deadzone = 10;
const int maxFlipSpeed = 10;
const int minFlipSpeed = 1;
const int flipThreshold = 15;

// Button state tracking
bool lastButtonState = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  clampServo1.attach(CLAMP_PIN1);
  setClampServo1(80); // Use new thresholded function
  clampServo2.attach(CLAMP_PIN2);
  clampServo2.write(100);

  flipServo.attach(CLAMP_PIN3);
  flipServo.write(flipCurrent);

  pinMode(enablePin, OUTPUT);
  pinMode(wheelServoIn1, OUTPUT);
  pinMode(wheelServoIn2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Setup complete. Waiting for button press...");
}

void loop() {
  bool currentButtonState = digitalRead(BUTTON_PIN);

  // Button pressed (edge trigger)
  if (lastButtonState == HIGH && currentButtonState == LOW && !flipping) {
    Serial.println("Button pressed - starting sequence");
    digitalWrite(LED_BUILTIN, HIGH);

    // Step 1: Lift clamps
    Serial.println("Lifting clamps");
    setClampServo1(160);
    clampServo2.write(0);
    delay(350);

    // Step 2: Spin motor
    Serial.println("Spinning wheel motor");
    digitalWrite(wheelServoIn1, HIGH);
    digitalWrite(wheelServoIn2, LOW);
    analogWrite(enablePin, 255);
    delay(1000);
    digitalWrite(wheelServoIn1, LOW);
    digitalWrite(wheelServoIn2, LOW);
    analogWrite(enablePin, 0);
    Serial.println("Motor stopped");

    // Step 3: Flip page down
    Serial.println("Beginning flip down");
    flipping = true;
    flipStart = flipCurrent;
    flipEnd = 0;
    lastFlipUpdate = millis();
  }

  lastButtonState = currentButtonState;

  // Handle flipping
  if (flipping && millis() - lastFlipUpdate >= flipInterval) {
    lastFlipUpdate = millis();

    if (abs(flipCurrent - flipEnd) <= deadzone) {
      flipCurrent = flipEnd;
      flipServo.write(flipCurrent);
      Serial.print("Reached position: ");
      Serial.println(flipEnd);

      if (flipEnd == 0 || flipEnd == 30) {
        Serial.println("Pausing at bottom position");
        delay(500);
        flipEnd = 190;
        Serial.println("Flipping back up");
      } else if (flipEnd == 190) {
        flipping = false;
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Flip complete. Resetting clamps");

        setClampServo1(80);
        clampServo2.write(100);
        delay(1000);
        Serial.println("Clamps reset. Waiting for next button press...");
      }
    } else {
      int distanceToTarget = abs(flipCurrent - flipEnd);
      int flipSpeed = (distanceToTarget < flipThreshold) ? minFlipSpeed : maxFlipSpeed;

      if (flipCurrent > flipEnd) {
        flipCurrent -= flipSpeed;
        if (flipCurrent < flipEnd) flipCurrent = flipEnd;
      } else {
        flipCurrent += flipSpeed;
        if (flipCurrent > flipEnd) flipCurrent = flipEnd;
      }

      flipServo.write(flipCurrent);
      Serial.print("Flipping... Current position: ");
      Serial.println(flipCurrent);
    }
  }
}

// Set clampServo1 only if change is greater than deadzone
void setClampServo1(int targetAngle) {
  if (lastClamp1Pos == -1 || abs(targetAngle - lastClamp1Pos) > clamp1Deadzone) {
    clampServo1.write(targetAngle);
    Serial.print("clampServo1 moved to ");
    Serial.println(targetAngle);
    lastClamp1Pos = targetAngle;
  } else {
    Serial.print("clampServo1 ignored small move to ");
    Serial.println(targetAngle);
  }
}
