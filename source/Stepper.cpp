/*
 * Stepper.cpp
 *
 *  Created on: 27.12.2021
 *      Author: Marcin
 */

#include "Stepper.h"

Stepper::Stepper(PinName A1, PinName A2, PinName B1, PinName B2, PinName enablePin, uint8_t noOfSteps) :
    phaseA1(A1, 1, true),   //PWM center aligned
    phaseA2(A2, 1, true),   //PWM center aligned
    phaseB1(B1, 1, true),   //PWM center aligned
    phaseB2(B2, 1, true),   //PWM center aligned
    enablePin(enablePin),
    noOfSteps(noOfSteps)
{
    static constexpr int PwmPeriodUs = 50;
    this->enablePin = 0;
    this->phaseA1.period_us(PwmPeriodUs);
    this->phaseA2.period_us(PwmPeriodUs);
    this->phaseB1.period_us(PwmPeriodUs);
    this->phaseA2.period_us(PwmPeriodUs);
}