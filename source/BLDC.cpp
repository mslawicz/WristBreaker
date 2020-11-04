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

// returns sine(argument)
// argument: 0..90 degrees
float MotorBLDC::fastSineD(float argument)
{
    static const float FullCycle = 360.0F;
    static const float HalfCycle = 180.0F;
    static const float QuarterCycle = 90.0F;
    float sign = 1.0F;
    
    if (argument < 0)
    {
        argument = -argument;
        sign = -sign;
    }

    if (argument >= FullCycle)
    {
        argument = std::fmod(argument, FullCycle);
    }

    if (argument >= HalfCycle)
    {
        argument -= HalfCycle;
        sign = -sign;
    }

    if (argument >= QuarterCycle)
    {
        argument = HalfCycle - argument;
    }

    int lowerIndex = static_cast<int>(argument);
    if (lowerIndex == (SineArraySize - 1))
    {
        return sign;
    }

    return sign * (SineLUT[lowerIndex] + (argument - static_cast<float>(lowerIndex)) * (SineLUT[lowerIndex + 1] - SineLUT[lowerIndex])); //NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
}