/*
 * BLDC.cpp
 *
 *  Created on: 04.11.2020
 *      Author: Marcin
 */

#include "BLDC.h"

MotorBLDC::MotorBLDC(PinName outA, PinName outB, PinName outC, PinName enable) :
    phaseA(outA),
    phaseB(outB),
    phaseC(outC),
    enable(enable)
{
    this->enable = 0;
}