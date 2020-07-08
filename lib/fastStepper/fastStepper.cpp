/*
A simple, lightweight library for running stepper motors via a hardware timer on an ESP32.
Can generate steps up to a few hundred kHz.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
*/

#include "fastStepper.h"

fastStepper::fastStepper(uint8_t stepPin, uint8_t dirPin, uint8_t timerNo, boolean revDir, void (*f)()) {
  _stepPin = stepPin;
  _dirPin = dirPin;
  _timerNo = timerNo;
  _revDir = revDir;
  timerFun = f;
}

void fastStepper::init() {
  pinMode(_stepPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  _timer = timerBegin(_timerNo, 2, true);
  timerAttachInterrupt(_timer,timerFun, true);
}

void IRAM_ATTR fastStepper::timerFunction() {
  // portENTER_CRITICAL_ISR(&timerMux);
  if (!pinState) {
    _step += dir; // Step is made on rising edge
    GPIO.out_w1ts = 1<<_stepPin;
    pinState = 1;
  } else {
    GPIO.out_w1tc = 1<<_stepPin;
    pinState = 0;
  }
  // portEXIT_CRITICAL_ISR(&timerMux);
}

// // A step is triggered on the rising edge of the step pin of the stepper driver.
// // Therefore, we use a 50% duty cycle, variable frequency PWM output.
// // Maximum step rate is 250 kHz. We also want some resolution left (to prevent sudden steps in speed).
// // Because of the pin toggling at 50% duty cycle, we need double the interrupt frequency.
// // Also, we want plenty resolution, to prevent discrete steps in stepper speed.


// Use speed as input variable?
// Use static member for microstep pins?
void fastStepper::update() {
  float absSpeed;
  uint32_t absSpeedInt;
  int8_t lastDir = dir;

  if (speed > 0) {
    dir = 1;
    absSpeed = speed*(microStep/16.0);
  } else {
    dir = -1;
    absSpeed = -speed*(microStep/16.0);
  }

  if (absSpeed>maxSpeed) absSpeed = maxSpeed; // Clip speed

  // Only update dir pin if changed (it doesn't change very often)
  if (dir != lastDir) {
    if(_revDir)
    {
      digitalWrite(_dirPin, dir==1 ? 0 : 1);
    }
    else
    {
      digitalWrite(_dirPin, dir==1 ? 1 : 0);
    }
  }

  if (lastSpeed!=speed) {
    if (absSpeed!=0) {
      absSpeedInt = (uint32_t) (400000.0/absSpeed);
      // Serial.println(absSpeedInt);
      timerAlarmWrite(_timer, absSpeedInt, true);
      if (!timerEnable) {
        timerAlarmEnable(_timer); // Re-enable timer
        timerEnable = 1;
      }
    } else {
      timerAlarmWrite(_timer, 100000, true);
      timerAlarmDisable(_timer);
      timerEnable = 0;
    }
  }
  lastSpeed = speed;
}

int32_t fastStepper::getStep() {
  int32_t step;
  portENTER_CRITICAL(&timerMux);
  step = _step;
  portEXIT_CRITICAL(&timerMux);
  return step;
}

void fastStepper::setStep(int32_t step) {
  portENTER_CRITICAL(&timerMux);
  _step = step;
  portEXIT_CRITICAL(&timerMux);
}
