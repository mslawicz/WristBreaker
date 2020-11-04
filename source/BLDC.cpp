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

// set motor stator magnetic field vector
// direction of stator magnetic field vector in degrees (== static rotor position)
// magnitude 0..1
void MotorBLDC::setFieldVector(float direction, float magnitude)
{
    static const float OneThirdCycle = 120.0F;
    if(magnitude < 0)
    {
        magnitude = 0.0F;
    }
    else if(magnitude > 1.0F)
    {
        magnitude = 1.0F;
    }

    // calculate normalized voltages (0..1) of stator windings
    float voltageA = 0.5F + 0.5F * fastSineD(direction);
    float voltageB = 0.5F + 0.5F * fastSineD(direction + OneThirdCycle);
    float voltageC = 1.5F - voltageA - voltageB;

    // drive PWM outputs with calculated voltages
    phaseA.write(magnitude * voltageA);
    phaseB.write(magnitude * voltageB);
    phaseC.write(magnitude * voltageC);
}