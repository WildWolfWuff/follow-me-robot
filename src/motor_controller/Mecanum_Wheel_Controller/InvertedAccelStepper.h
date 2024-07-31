#ifndef INVERTED_ACCEL_STEPPER_H
#define INVERTED_ACCEL_STEPPER_H

#include <AccelStepper.h>

class InvertedAccelStepper : public AccelStepper {
public:
  InvertedAccelStepper(uint8_t interface = AccelStepper::DRIVER, uint8_t pin1 = 2, uint8_t pin2 = 3, uint8_t pin3 = 4, uint8_t pin4 = 5, bool enable = true)
    : AccelStepper(interface, pin1, pin2, pin3, pin4, enable) {}

  void setSpeed(float speed) {
    AccelStepper::setSpeed(-speed);
  }
};

#endif // INVERTED_ACCEL_STEPPER_H