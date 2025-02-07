#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

#include "pico/stdlib.h"
#include <hardware/pwm.h>

#include "ncr_common.h"

#define CAN_ID 0x402
#define SERVO_1_ID 0x60
#define SERVO_2_ID 0x61

#define DEG_CLOSED 140
#define DEG_OPEN 230

#define PIN_SERVO_1 A0
#define PIN_SERVO_2 A2
#define NUM_SERVOS 2

#define SERVO_DEG_MIN 0
#define SERVO_DEG_MAX 270

#define TOP 65535
#define CC_LOW 6553
#define CC_HIGH 65535u>>1
#define DIV_INT 9
#define DIV_FRAC 9

void move_servo(uint servo_pin, uint degree);
void on_can_received(int packet_size);

uint servos[NUM_SERVOS] = { PIN_SERVO_1, PIN_SERVO_2 };

Adafruit_MCP2515 can(PIN_CAN_CS);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);

  // Initialize CAN transceiver
  while(!can.begin(CAN_BAUD)) {
    Serial.println("Error initializing MCP2515.");
    delay(100);
  }
  Serial.println("MCP2515 found.");
  can.onReceive(PIN_CAN_INTERRUPT, on_can_received);

  // Configure GPIO and PWM slices
  for (size_t i=0; i<NUM_SERVOS; i++) {
    gpio_set_function(servos[i], GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(servos[i]);
    pwm_set_clkdiv_int_frac4(slice, DIV_INT, DIV_FRAC);
    pwm_set_enabled(slice, true);
  }

  for (size_t i=0; i<NUM_SERVOS; i++) {
    move_servo(servos[i], DEG_CLOSED);
  }
  
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() { }

void move_servo(uint servo_pin, uint degree) {
  uint16_t cc = map(degree, SERVO_DEG_MIN, SERVO_DEG_MAX, CC_LOW, CC_HIGH);
  pwm_set_gpio_level(servo_pin, cc);
}

void on_can_received(int packet_size) {
  uint16_t id = can.packetId();
  PACKET_TYPE packet_type = (PACKET_TYPE)can.read();
  switch (packet_type) {
    case VALVE_COMMAND_PACKET: {
      uint16_t id;
      can.readBytes((uint8_t*)&id, 2);
      bool position = can.read();

      digitalWrite(LED_BUILTIN, position);

      if (id == SERVO_1_ID) { 
        move_servo(PIN_SERVO_1, position ? DEG_OPEN : DEG_CLOSED);
      } else if (id == SERVO_2_ID) {
        move_servo(PIN_SERVO_2, position ? DEG_OPEN : DEG_CLOSED);        
      }
    } break;
  }
}