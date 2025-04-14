#include <Servo.h>

#define BUTTON_PIN 2 // Button connected to pin 2 {pins 2 reserved for inputs; eg 3 for voice recog.}

#define CLAMP_PIN1 3
#define CLAMP_PIN2 5
#define CLAMP_PIN3 6 

const int wheelServoIn1 = 7;  // Wheel servo on pin 7
const int wheelServoIn2 = 8;  // Wheel servo on pin 8
const int enablePin = 4; //pwm pin

const int trigPin = 14;
const int echoPin = 15;

float duration, distance;
Servo clampServo1;
Servo clampServo2;
Servo flipServo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP) ; // Internal pull-up resistor #input

  clampServo1.attach(CLAMP_PIN1);
  clampServo1.write(0); //start at off position
  clampServo2.attach(CLAMP_PIN2);
  clampServo2.write(0);
  flipServo.attach(CLAMP_PIN3);
  flipServo.write(0);

  pinMode(enablePin, OUTPUT);
  pinMode(wheelServoIn1, OUTPUT);
  pinMode(wheelServoIn2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  //----------------------
  // put your main code here, to run repeatedly:
  
  if (digitalRead(BUTTON_PIN) == HIGH) { // Button is pressed
    Serial.print("Button pressed");

    //lift clamps
    clampServo1.write(90);  // Move servo to 90 degrees
    clampServo2.write(90);
    delay(1000);
    /* //do we have to drop servos down after lift?
    clampServo1.write(0);  // Move servo back 0 degrees(off position)
    clampServo2.write(0); */

    //turn wheel
    digitalWrite(wheelServoIn1, HIGH); // Move forward at moderate speed
    digitalWrite(wheelServoIn2, LOW);
    delay(1200);           // Run for 2 seconds
    analogWrite(enablePin, 210);  // Speed (0-255)
    analogWrite(enablePin, 0); //turn motor off

    //Raise flipper 
    flipServo.write(90);
    delay(2000);
    flipServo.write(0);

    //clamp pages down
    clampServo1.write(0);   // Move servo to 0 degrees
    clampServo2.write(0);
    delay(1000);

  }
  
}
