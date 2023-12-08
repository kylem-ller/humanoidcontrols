#include <Arduino.h>
#include "Controller.h"

#define BTN 34
#define LED_PIN 13
#define POT 39

// State Machine
enum Case { IDLE, CALIBRATE, BALANCE };
Case c = IDLE;
volatile bool buttonIsPressed = false;

// Controller   
Controller controller;
const float pi = 3.14159;

// Timers
int mainTimer;
int buttonTimer;
int calibrateTimer;
int i = 0;
/*hw_timer_t * imuTimer = NULL;
hw_timer_t * motorTimer = NULL;
// portMUX_TYPE imuTimerMux = portMUX_INITIALIZER_UNLOCKED;*/

//Initialization ------------------------------------
void IRAM_ATTR buttonISR() {
  buttonIsPressed = true;
}

/*void IRAM_ATTR imuISR() {
  portENTER_CRITICAL_ISR(&imuTimerMux);
  controller.imu.update();
  portEXIT_CRITICAL_ISR(&imuTimerMux);
}*/

void setup() {
  Serial.begin(9600);
  pinMode(BTN, INPUT);
  pinMode(POT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(BTN, buttonISR, RISING);

  mainTimer = millis();
  buttonTimer = millis();

  /*imuTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(imuTimer, &imuISR, true);
  timerAlarmWrite(imuTimer, 50000, true); // 50000 ticks -> 50 ms
  timerAlarmEnable(imuTimer);*/

  controller.init();
}

void loop() {
  controller.imu.update();

  switch (c) {
    case IDLE:
      if (checkForButtonPress()) { // Check if button has been pressed
        toCalibrate(); // Change state to calibrate
      }
      controller.stopActuators();
      break;

    case CALIBRATE:
      if (checkForButtonPress()) { // Check if button has been pressed
        toIdle(); // Change state to idle
      }
      flashLight(); // Blink the LED
      controller.stopActuators();
      if (pastTime(calibrateTimer, 3000)) { // Check if 3 seconds has passed in calibrate mode
        toBalance(); // Change state to balancing
      }
      break;

    case BALANCE:
      if (checkForButtonPress() || controller.falling()) { // Check if button has been pressed, or if the bot has tilted too much
        toIdle(); // Change state to idle (turn off actuators)
      }
      if (pastTime(mainTimer, 10)) {
        while(!pastTime(mainTimer, 12)) {}
        mainTimer = millis();

        if (i <= 0) {
          controller.LW.update();
          controller.RW.update();
          controller.A2.update();
          i += 4;
        }
        float potVal = analogRead(POT);
        float targetPos = ((potVal/4095-0.5) / 5);
        // Serial.println(targetPos);
        float target[6] = {targetPos, 0, 0, 0, 0, 0};
        controller.setTarget(target); // Run actuators to stabilize at position & velocity targets
      }
      break;

    
  }
}

bool checkForButtonPress() {
  if (!pastTime(buttonTimer, 1000)) {
    buttonIsPressed = false;
  }
  if (buttonIsPressed) {
    buttonTimer = millis();
    buttonIsPressed = false;
    return true;
  }
  return false;
}

bool pastTime(int timer, int time) {
  return millis() - timer > time;
}

void toIdle() {
  c = IDLE;
  ledOff();
  controller.stopActuators();
}

void toBalance() {
  c = BALANCE;
  ledOn();
  controller.zeroState();
}

void toCalibrate() {
  c = CALIBRATE;
  calibrateTimer = millis();
}

void flashLight() {
  if ((millis() - calibrateTimer) / 500 % 2) {
    ledOn();
  } else {
    ledOff();
  }
}

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
}