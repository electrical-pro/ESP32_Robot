/*
A simple, lightweight library for running stepper motors via a hardware timer on an ESP32.
Can generate steps up to a few hundred kHz.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
*/

#ifndef FASTSTEPPER_H
#define FASTSTEPPER_H

#include <Arduino.h>

extern portMUX_TYPE timerMux;

class fastStepper {
public:
  fastStepper(uint8_t stepPin, uint8_t dirPin, uint8_t timerNo, boolean revDir, void (*f)());
  void timerFunction();
  void init();
  void update();
  int32_t getStep();
  void setStep(int32_t step);

  float speed = 0;
  float maxSpeed = 3000;

  uint8_t microStep;
private:
  void (*timerFun)();
  volatile boolean pinState = 0;
  volatile int32_t _step = 0; // Step counter
  volatile int8_t dir = 1;
  uint8_t _stepPin;
  uint8_t _dirPin;
  hw_timer_t* _timer;
  uint8_t _timerNo;
  boolean _revDir;
  float lastSpeed = 0.0;
  boolean timerEnable = 0;
  // static void (*timerFunction)();
  // static const portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
};



#endif
