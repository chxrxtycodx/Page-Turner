#include <Servo.h>

#define BUTTON_PIN 2 // Button connected to pin 3 {pins 2 & 3 reserved for inputs}

#define CLAMP_PIN1 9
#define CLAMP_PIN2 4
#define CLAMP_PIN3 5 

#define SERVO_PIN1 6  // Wheel servo on pin 6
#define SERVO_PIN2 7  // Wheel servo on pin 7

Servo clampServo1;
Servo clampServo2;
Servo clampServo3;

Servo wheelServo1;
Servo wheelServo2;

void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON_PIN, INPUT_PULLUP) ; // Internal pull-up resistor #input

  clampServo1.attach(CLAMP_PIN1);
  clampServo2.attach(CLAMP_PIN2);
  clampServo3.attach(CLAMP_PIN3);

  wheelServo1.attach(SERVO_PIN1);
  wheelServo2.attach(SERVO_PIN2);

  // wheelServo.write(90); // Start in STOP position

  //lift clamp
  clampServo1.write(90);  // Move servo to 90 degrees
  // delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  // if (digitalRead(BUTTON_PIN) == HIGH) { // Button is pressed
  //   Serial.println("Button pressed");
  //   //lift clamp
  //   clampServo.write(90);  // Move servo to 90 degrees
  //   delay(1000);

  //   //turn wheel
  //   wheelServo.write(HIGH); // Move forward at moderate speed
  //   delay(2000);           // Run for 5 seconds
  //   wheelServo.write(LOW);  // Stop the servo
  //   delay(3000);

  //   //Raise flipper 


  //   //clamp down
  //   clampServo.write(0);   // Move servo to 0 degrees
  //   delay(1000);

  // }
  
}
