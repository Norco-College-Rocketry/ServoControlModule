#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

#include "ncr_common.h"

#define SERVO_1_PIN 5
#define SERVO_2_PIN 6

int map_270_servo(int);

Servo servo;

Adafruit_MCP2515 can(10);

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);

  if (!can.begin(CAN_BAUD)) {
   
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 found."); 

  servo.attach(SERVO_1_PIN, 500, 2500);
}

void loop() {
  servo.write(120);
  delay(5000);
  servo.write(map_270_servo(0));
  delay(5000);
}

int map_270_servo(int angle) {
  return map(angle, 0, 270, 0, 180);
}