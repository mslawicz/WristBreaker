/*
 * BLDC.cpp
 *
 *  Created on: 04.11.2020
 *      Author: Marcin
 */

#include "BLDC.h"

MotorBLDC::MotorBLDC(PinName outA, PinName outB, PinName outC, PinName enable, uint8_t noOfPoles) :
    phaseA(outA),
    phaseB(outB),
    phaseC(outC),
    enable(enable),
    noOfPoles(noOfPoles)
{
    static const int PwmPeriodUs = 100;
    this->enable = 0;
    this->phaseA.period_us(PwmPeriodUs);
    this->phaseB.period_us(PwmPeriodUs);
    this->phaseC.period_us(PwmPeriodUs);
}

// returns sine(argument)
// argument in degrees
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

    // at this stage the argument is in the range 0..90

    int lowerIndex = static_cast<int>(argument);
    if (lowerIndex == (SineArraySize - 1))
    {
        return sign;
    }

    return sign * (SineLUT[lowerIndex] + (argument - static_cast<float>(lowerIndex)) * (SineLUT[lowerIndex + 1] - SineLUT[lowerIndex])); //NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
}

// set motor stator magnetic field vector
// electricAngle - angle of stator magnetic field vector in degrees (== requested rotor position within electric cycle)
// magnitude 0..1
void MotorBLDC::setFieldVector(float electricAngle, float magnitude)
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

    // calculate normalized voltage level (0..1) of stator windings
    static const float VoltageMeanLevel = 0.5F;
    //static const float TripleVoltageMeanLevel = 3 * VoltageMeanLevel;
    float voltageA = VoltageMeanLevel + VoltageMeanLevel * magnitude * fastSineD(electricAngle - OneThirdCycle);
    float voltageB = VoltageMeanLevel + VoltageMeanLevel * magnitude * fastSineD(electricAngle);
    float voltageC = VoltageMeanLevel + VoltageMeanLevel * magnitude * fastSineD(electricAngle + OneThirdCycle);

    // drive PWM outputs with calculated voltage levels
    phaseA.write(voltageA);
    phaseB.write(voltageB);
    phaseC.write(voltageC);
}