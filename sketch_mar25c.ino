// Define motor control pins
const int enablePin = 9;
const int motorIn1 = 2;
const int motorIn2 = 3;

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Turn on motor - FORWARD direction
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  analogWrite(enablePin, 210);  // Speed (0-255)
  delay(1000)
  analogWrite(enablePin, 0)
}

void loop() {
  // Motor will keep spinning forward
}
