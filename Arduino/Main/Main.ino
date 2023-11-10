#include <Arduino.h>
#include "Controller.h"
#include "Wire.h"
#include <arduino-timer.h>

#define BTN 12
#define LED_PIN 13

//Setup variables ------------------------------------
const long pi = 3.14159;
const int freq = 5000;
const int pwmChannel = 0;
const int resolution = 8;
volatile bool buttonIsPressed = false;

// State Machine
enum Case { IDLE, CALIBRATE, STABLE_DRIVE };
Case c = IDLE;

// Controller
Controller controller;

// Timers
long buttonTimer;
long calibrateTimer;
auto imuTimer = timer_create_default();
auto motorTimer = timer_create_default();

//Initialization ------------------------------------
void IRAM_ATTR isr() {
  buttonIsPressed = true;
}

void setup() {
  pinMode(BTN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(BTN, isr, RISING);

  ledcSetup(pwmChannel, freq, resolution);

  buttonTimer = millis();
  imuTimer.every(50, updateIMU);

  Wire.begin();
  delay(10);
  controller.init();
}

void loop() {
  delay(100);
  switch (c) {
    case IDLE:
      if (checkForButtonPress()) {
        c = CALIBRATE;
        calibrateTimer = millis();
      }
      break;
    case CALIBRATE:
      if (checkForButtonPress()) {
        c = IDLE;
      }
      if ((millis() - buttonTimer) / 250 % 2) {
        led_on();
      } else {
        led_off();
      }
      if (millis() - buttonTimer > 3000) {
        c = STABLE_DRIVE;
        controller.zeroState();
      }
      break;
    case STABLE_DRIVE:
      if (checkForButtonPress() || controller.falling()) {
        c = IDLE;
        controller.stopActuators();
      }
      controller.updateState();
      long target[6] = {0, 0, 0, 0, pi, 0};
      controller.setTarget(target);
      break;
  }
}

bool checkForButtonPress() {
  if (millis() - buttonTimer > 50) {
    buttonIsPressed = false;
  }
  if (buttonIsPressed) {
    buttonTimer = millis();
    buttonIsPressed = false;
    return true;
  }
  return false;
}

void led_on() {
  digitalWrite(LED_PIN, HIGH);
}

void led_off() {
  digitalWrite(LED_PIN, LOW);
}

void updateLW() {
  controller.LW->update();
}

void updateRW() {
  controller.RW->update();
}

void updateLA2() {
  controller.LA2->update();
}

void updateRA2() {
  controller.RA2->update();
}

bool updateIMU(void *) {
  controller.imu.update();
  return true;
}