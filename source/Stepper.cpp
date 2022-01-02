/*
 * Stepper.cpp
 *
 *  Created on: 27.12.2021
 *      Author: Marcin
 */

#include "Stepper.h"
#include "Convert.h"
#include "Logger.h"

//global variables for test
float g_stp[5];

Stepper::Stepper(PinName A1, PinName A2, PinName B1, PinName B2, PinName enablePin, uint8_t noOfPolePairs) :
    phaseA1(A1, 1, true),   //PWM center aligned
    phaseA2(A2, 1, true),   //PWM center aligned
    phaseB1(B1, 1, true),   //PWM center aligned
    phaseB2(B2, 1, true),   //PWM center aligned
    enablePin(enablePin),
    noOfPolePairs(noOfPolePairs)
{
    static constexpr int PwmPeriodUs = 50;
    this->enablePin = 0;
    this->phaseA1.period_us(PwmPeriodUs);
    this->phaseA2.period_us(PwmPeriodUs);
    this->phaseB1.period_us(PwmPeriodUs);
    this->phaseB2.period_us(PwmPeriodUs);
    electricPeriod = 1.0F / static_cast<float>(noOfPolePairs);
}

// set motor stator magnetic field vector
// electricAngle - angle of stator magnetic field vector in degrees (== requested rotor position within electric cycle)
// magnitude 0..1
void Stepper::setFieldVector(float electricAngle, float magnitude)
{
    if(magnitude < 0)
    {
        magnitude = 0.0F;
    }
    else if(magnitude > 1.0F)
    {
        magnitude = 1.0F;
    }

    // calculate PWM duty for stator winding voltages
    double pwmDuty = magnitude * fastSine(electricAngle);
    double pwmDutyA1 = (pwmDuty > 0) ? pwmDuty : 0;
    double pwmDutyA2 = (pwmDuty < 0) ? -pwmDuty : 0;

    pwmDuty = magnitude * fastSine(electricAngle + QuarterCycle);
    double pwmDutyB1 = (pwmDuty > 0) ? pwmDuty : 0;
    double pwmDutyB2 = (pwmDuty < 0) ? -pwmDuty : 0;

    // drive PWM outputs with sine PWM duties
    phaseA1.write(pwmDutyA1);
    phaseA2.write(pwmDutyA2);
    phaseB1.write(pwmDutyB1);
    phaseB2.write(pwmDutyB2);
}

//calibrate stepper motor
//to be called periodically until returns true
//returns true when calibration is complete
bool Stepper::calibrate(float encoderPosition)
{
    float encoderPhase = FullCycle * static_cast<float>(noOfPolePairs) * fmodf(encoderPosition, electricPeriod);
    float phaseShift = cropAngle(FullCycle + currentPhase - encoderPhase);

    static uint32_t cnt = 0;
    if(++cnt % 1000 == 0)
    {
        LOG_INFO("enc=" << encoderPosition << "  cPh=" << currentPhase << "  encPh=" << encoderPhase << "  dPh=" << phaseShift); 
    }

    currentPhase += 0.5F;
    if(currentPhase > FullCycle)
    {
        currentPhase -= FullCycle;
    }
    static AnalogIn tPot(PA_5); float torque = tPot.read(); //XXX test; also use PA_6 and PA_7

    g_stp[0] = encoderPosition;
    g_stp[1] = fmodf(encoderPosition, electricPeriod);
    g_stp[2] = currentPhase;
    g_stp[3] = encoderPhase;
    g_stp[4] = phaseShift;

    setFieldVector(currentPhase, torque);

    return false;
}
