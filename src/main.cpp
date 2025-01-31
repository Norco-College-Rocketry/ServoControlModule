#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

#include "pico/stdlib.h"
#include <hardware/pwm.h>

#include "ncr_common.h"

#define PID_LABEL 1

#define PIN_SERVO_1 A0
#define PIN_SERVO_2 A2

#define NUM_SERVOS 2

#define TOP 65535
#define CC_LOW 6553
#define CC_HIGH 65535u>>1
#define DIV_INT 9
#define DIV_FRAC 9

void move_servo(uint servo_pin, uint degree);
int map_270_servo(int);
void on_can_received(int packet_size);

// Servo servos[NUM_SERVOS];

Adafruit_MCP2515 can(10);
uint slice;

void setup() {
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);
  gpio_set_function(PIN_SERVO_1, GPIO_FUNC_PWM);
  slice = pwm_gpio_to_slice_num(PIN_SERVO_1);
  pwm_set_gpio_level(PIN_SERVO_1, CC_LOW);
  pwm_set_clkdiv_int_frac4(slice, DIV_INT, DIV_FRAC);
  pwm_set_enabled(slice, true);

  // Serial.begin(9600);

/*   while(!can.begin(CAN_BAUD)) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Error initializing MCP2515.");
    delay(100);
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("MCP2515 found.");  */

/*   servos[0].attach(PIN_SERVO_1, 500, 2500);
  servos[1].attach(PIN_SERVO_2, 500, 2500); */

  // can.onReceive(PIN_CAN_INTERRUPT, on_can_received);
}

void loop() {
  move_servo(PIN_SERVO_1, 135);
  delay(2000);
  move_servo(PIN_SERVO_1, 225);
  delay(2000);
          
  // servos[0].write(120);
  // delay(5000);
  // digitalWrite(LED_BUILTIN, HIGH);
  // servos[0].write(map_270_servo(0));
  // delay(5000);
  // digitalWrite(LED_BUILTIN, LOW);
}

void move_servo(uint servo_pin, uint degree) {
  uint16_t cc = map(degree, 0, 270, CC_LOW, CC_HIGH);
  pwm_set_gpio_level(servo_pin, cc);
}

int map_270_servo(int angle) {
  return map(angle, 0, 270, 0, 180);
}

void on_can_received(int packet_size) {
  // TODO parse & execute valve command
}