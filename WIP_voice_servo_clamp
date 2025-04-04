/**
  ******************************************************************************
  * @file    vr_sample_control_led.ino
  * @author  JiapengLi
  * @brief   This file provides a demostration on 
              how to control led by using VoiceRecognitionModule
  ******************************************************************************
  * @note:
        voice control led
  ******************************************************************************
  * @section  HISTORY
    
    2013/06/13    Initial version.
  */
  
#include <Servo.h> // ADDED

#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

/**        
  Connection
  Arduino    VoiceRecognitionModule
   2   ------->     TX
   3   ------->     RX
*/
VR myVR(2,3);    // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7]; // save record
uint8_t buf[64];

int led = 13;

#define onRecord    (0) // julian "turn"
#define offRecord   (2) //changed from 1 , julian "off"
#define onRecord1 (1) // ashley "turn"
#define offRecord1 (3) // ashley "off"

#define clamp_pin1 (9) // ADDED

Servo ClampServo1;
bool servoActive = false;
unsigned long servoStartTime = 0;
const int clampHoldTime = 3000; // milliseconds

//Less "angry" servo
int16_t servoCurrent=0, servoDesired=0, servoDelta=7;

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf     --> command length
           len     --> number of parameters
*/
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

void setup()
{
  /** initialize */
  myVR.begin(9600);
  
  Serial.begin(115200);
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
  pinMode(led, OUTPUT);
    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)onRecord) >= 0){
    Serial.println("onRecord loaded");
  }

   if(myVR.load((uint8_t)onRecord1) >= 0){ // ADDED THIS PART
    Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
    Serial.println("offRecord loaded");
  }

  if(myVR.load((uint8_t)offRecord1) >= 0){ // ADDED THIS PART
    Serial.println("offRecord loaded");
  }

  ClampServo1.attach(clamp_pin1); // Attach servo to pin D9
  ClampServo1.write(150);           // Set servo to initial (open) position
  servoCurrent=150;
  servoDesired=150;
  //ADDED THIS PART

}

void loop()
{
  
  int ret = myVR.recognize(buf, 50);

  // If a voice command is recognized
  if (ret > 0) {
    switch (buf[1]) {
      case onRecord:
      case onRecord1:
        // Turn on LED checkpoint
        digitalWrite(led, HIGH);
        Serial.println("Voice trigger received. Clamping...");

        // Activate clamp
        servoDesired=30;// Adjust angle if needed
        
        servoActive = true;
        servoStartTime = millis(); // Start timer
        break;

      case offRecord:
      case offRecord1:
        // Optional off command (e.g. to reset LED)
        digitalWrite(led, LOW);
        break;

      default:
        Serial.println("Record function undefined");
        break;
    }

    // Debug print
    printVR(buf);
  }

  // Handle servo auto-release after 3 seconds
  if (servoActive && (millis() - servoStartTime >= clampHoldTime)) {
    //Schedule a smooth return to open
    servoDesired=150;

    servoActive = false;

    Serial.println("Servo released.");
    digitalWrite(led, LOW); // Optional: turn off LED after release
  }

  if (servoCurrent < servoDesired)
  {
    if (servoCurrent+servoDelta > servoDesired)
      servoCurrent=servoDesired;
    else servoCurrent+=servoDelta;
    ClampServo1.write(servoCurrent);
  }
  else if (servoCurrent > servoDesired)
  {
    if (servoCurrent-servoDelta < servoDesired)
      servoCurrent=servoDesired;
    else servoCurrent-=servoDelta;
    ClampServo1.write(servoCurrent);
  }
}



