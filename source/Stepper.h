/*
 * Stepper.h
 *
 *  Created on: 27.12.2021
 *      Author: Marcin
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include "Motor.h"
#include "FastPWM.h"
#include <mbed.h>

class Stepper : public Motor
{
public:
    Stepper(PinName A1, PinName A2, PinName B1, PinName B2, PinName enablePin, uint8_t noOfPolePairs);
    void setFieldVector(float electricAngle, float magnitude);
private:
    FastPWM phaseA1;
    FastPWM phaseA2;
    FastPWM phaseB1;
    FastPWM phaseB2;
    DigitalOut enablePin;
    uint8_t noOfPolePairs;  // motor number of electric pole pairs (electric to mechanical revolution ratio)
};

#endif /* STEPPER_H_ */