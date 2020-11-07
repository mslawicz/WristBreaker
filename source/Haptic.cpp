/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include <iostream> //XXX

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    float positionMin,      // minimal value of motor position
    float positionMax      // maximum value of motor position
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    positionMin(positionMin),
    positionMax(positionMax)
{
    pMotor->setEnablePin(1);
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
}

// set motor torque vector
// direction - -1 maximum reverse, 0 hold position, 1 maximum forward
// magnitude - torque vector magnitude 0..1 (PWM wave multiplier)
void HapticDevice::setTorqueVector(float  direction, float  magnitude)
{
    static const float FullCycle = 360.0F;      // full electric cycle in degrees
    static const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees

    static int cnt = 0;
    positionSens = pEncoder->getValue();
    pMotor->setFieldVector(0, magnitude);
    if(cnt++ % 100 == 0)
    {
        //std::cout << magnitude << "  " << positionSens << std::endl;
    }

    float targetElectricAngle = fmodf(positionSens, positionPeriod) * FullCycle / positionPeriod// encoder position cycle phase (0..360 degrees)
        + electricCyclePhaseShift   // constant phase shift between encoder and motor cycle phase (0..360 degrees)
        + direction * QuarterCycle; // additional phase shift for desired torque vector (-90 .. 90 degrees)
    pMotor->setFieldVector(targetElectricAngle, magnitude); // set motor stator magnetic field vector
}