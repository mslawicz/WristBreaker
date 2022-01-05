/*
 * BLDC.cpp
 *
 *  Created on: 04.11.2020
 *      Author: Marcin
 */

#include "BLDC.h"
#include "Convert.h"
#include "Logger.h"
#include <cstdint>

//global variables for test
float g_bldc[5];

MotorBLDC::MotorBLDC(PinName outA, PinName outB, PinName outC, PinName enablePin, uint8_t noOfPolePairs) :
    phaseA(outA, 1, true),  //PWM center aligned
    phaseB(outB, 1, true),  //PWM center aligned
    phaseC(outC, 1, true),  //PWM center aligned
    enablePin(enablePin),
    noOfPolePairs(noOfPolePairs)
{
    static constexpr int PwmPeriodUs = 50;  //50 us -> PWM frequency 20 kHz
    this->enablePin = 0;
    this->phaseA.period_us(PwmPeriodUs);
    this->phaseB.period_us(PwmPeriodUs);
    this->phaseC.period_us(PwmPeriodUs);
    electricPeriod = 1.0F / static_cast<float>(noOfPolePairs);
}

// returns space vector modulation value
// argument in degrees
float MotorBLDC::getSvmValue(float argument)
{
    float sign = 1.0F;

    if (argument < 0)
    {
        argument = -argument;
        sign = -sign;
    }

    if (argument >= FullCycle)
    {
        argument = std::fmodf(argument, FullCycle);
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
    if(magnitude < 0)
    {
        magnitude = 0.0F;
    }
    else if(magnitude > 1.0F)
    {
        magnitude = 1.0F;
    }

    // calculate PWM duty for stator winding voltages
    static constexpr float halfDuty = 0.5F;
    double pwmDutyA = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle - OneThirdCycle);
    double pwmDutyB = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle);
    double pwmDutyC = halfDuty + halfDuty * magnitude * getSvmValue(electricAngle + OneThirdCycle);

    // drive PWM outputs with calculated PWM duties
    phaseA.write(pwmDutyA);
    phaseB.write(pwmDutyB);
    phaseC.write(pwmDutyC);
}

//calibration procedure setup
//to be called before calling calibrate()
void MotorBLDC::calibrationSetup()
{
    currentPhase = 0.0F;
    magnitude = 0.0F;
    calibrationPhaseStep = 2 * FullCycle * actuatorData.calibrationRange / electricPeriod / static_cast<float>(actuatorData.noOfCalibrationSteps);
    dPhase = calibrationPhaseStep;
}

//calibrate BLDC motor
//to be called periodically until returns true
//returns true when calibration is complete
bool MotorBLDC::calibrate()
{
    float encoderPhase = FullCycle * static_cast<float>(noOfPolePairs) * fmodf(actuatorData.encoderValue, electricPeriod);
    float phaseShift = cropAngle(FullCycle + currentPhase - encoderPhase);
    //const float PhaseStep = electricPeriod * 10.0F;

    static uint32_t cnt = 0;
    if(++cnt % 1000 == 0)
    {
        LOG_INFO("pos=" << actuatorData.position << "  enc=" << actuatorData.encoderValue << "  cPh=" << currentPhase << "  encPh=" << encoderPhase << "  dPh=" << phaseShift); 
    }

    if(actuatorData.position < -actuatorData.calibrationRange)
    {
        dPhase = calibrationPhaseStep;
    }
    if(actuatorData.position > actuatorData.calibrationRange)
    {
        dPhase = -calibrationPhaseStep;
    }    
    currentPhase += dPhase;
    if(currentPhase > FullCycle)
    {
        currentPhase -= FullCycle;
    }
    static AnalogIn tPot(PA_5); float torque = tPot.read(); //XXX test; also use PA_6 and PA_7
    actuatorData.calibrationMagnitude = torque; //XXX test

    g_bldc[0] = actuatorData.encoderValue;
    g_bldc[1] = fmodf(actuatorData.encoderValue, electricPeriod);
    g_bldc[2] = currentPhase;
    g_bldc[3] = encoderPhase;
    g_bldc[4] = phaseShift;

    //gradually increase field vector magnitude
    constexpr float MagnitudeStep = 0.001F;
    magnitude += MagnitudeStep * actuatorData.calibrationMagnitude;    // 0.1% increase at a time
    if(magnitude > actuatorData.calibrationMagnitude)
    {
        magnitude = actuatorData.calibrationMagnitude;
    }

    setFieldVector(currentPhase, magnitude);
    
    return false;
}