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
    Stepper(PinName A1, PinName A2, PinName B1, PinName B2, PinName enablePin, uint8_t noOfSteps); 
private:
    FastPWM phaseA1;
    FastPWM phaseA2;
    FastPWM phaseB1;
    FastPWM phaseB2;
    DigitalOut enablePin;
    uint8_t noOfSteps;  // number of steps (divide by 4 to get electric revolutions)
};

#endif /* STEPPER_H_ */