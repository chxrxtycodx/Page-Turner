#include <Servo.h>

#define BUTTON_PIN 2   // Switch connected to pin 2
#define SERVO_PIN 9    // Servo connected to pin 9

Servo myServo;

void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Enable internal pull-up resistor
    myServo.attach(SERVO_PIN);          // Attach servo to pin 9
    myServo.write(0);                   // Start at 0 degrees
}

void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) { // Wait until button is pressed
        myServo.write(90); // Move to 50% of full range (0° to 90°)
        delay(1000); // Wait for movement to complete
    }
}
