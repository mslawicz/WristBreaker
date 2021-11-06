/*
 * BDC.cpp
 *
 *  Created on: 06.11.2021
 *      Author: Marcin
 */

#include "BDC.h"
#include <cstdint>

MotorDC::MotorDC(PinName outA, PinName outB) :
    phaseA(outA, 1, true),  //PWM center aligned
    phaseB(outB, 1, true)   //PWM center aligned
{
    static constexpr int PwmPeriodUs = 50;
    this->phaseA.period_us(PwmPeriodUs);
    this->phaseB.period_us(PwmPeriodUs);
}