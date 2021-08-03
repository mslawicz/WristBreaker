/*
 * BLDC.cpp
 *
 *  Created on: 04.11.2020
 *      Author: Marcin
 */

#include "BLDC.h"
#include <cstdint>

MotorBLDC::MotorBLDC(PinName outA, PinName outB, PinName outC, PinName enable, uint8_t noOfPoles) :
    phaseA(outA, 1, true),  //PWM center aligned
    phaseB(outB, 1, true),  //PWM center aligned
    phaseC(outC, 1, true),  //PWM center aligned
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
float MotorBLDC::getSvmValue(float argument)
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
    if (lowerIndex == (LutSize - 1))
    {
        return sign * SvmLUT[lowerIndex];   //NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    }

    return sign * (SvmLUT[lowerIndex] + (argument - static_cast<float>(lowerIndex))* (SvmLUT[lowerIndex + 1] - SvmLUT[lowerIndex])); //NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
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

    // calculate PWM duty for stator winding voltages
    static const float halfDuty = 0.5F;
    double pwmDutyA = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle - OneThirdCycle);
    double pwmDutyB = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle);
    double pwmDutyC = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle + OneThirdCycle);
    // double pwmDutyA = magnitude * halfDuty * (1 + getSvmValue(electricAngle - OneThirdCycle));
    // double pwmDutyB = magnitude * halfDuty * (1 + getSvmValue(electricAngle));
    // double pwmDutyC = magnitude * halfDuty * (1 + getSvmValue(electricAngle + OneThirdCycle));    

    // drive PWM outputs with calculated PWM duties
    phaseA.write(pwmDutyA);
    phaseB.write(pwmDutyB);
    phaseC.write(pwmDutyC);
}