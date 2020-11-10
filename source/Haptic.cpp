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
    positionPeriod = 1.0F / static_cast<float>(pMotor->getNoOfPoles());
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
}

// set motor torque
// torque: -1 maximum reverse, 0 hold position, 1 maximum forward
void HapticDevice::setTorque(float torque)
{
    static const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    static const float FullCycle = 360.0F;      // full electric cycle in degrees
    static const float HalfPositionRange = 0.5F;    // half of the full encoder range

    static int cnt = 0; //XXX test
    float newPositionSens = pEncoder->getValue();
    float deltaPositionSens = newPositionSens - positionSens;
    if(deltaPositionSens < -HalfPositionRange)
    {
        deltaPositionSens += 1.0F;
    }
    else if(deltaPositionSens > HalfPositionRange)
    {
        deltaPositionSens -= 1.0F;
    }
    currentPhase += deltaPositionSens * FullCycle / positionPeriod;
    if(currentPhase < 0)
    {
        currentPhase += FullCycle;
    }
    else if(currentPhase > FullCycle)
    {
        currentPhase -= FullCycle;
    }


    if(cnt++ % 127 == 0)
    {
        std::cout << "pos=" << positionSens << "  " << newPositionSens << "  " << deltaPositionSens ;
        std::cout << "   Phase=" << currentPhase << "  trq=" << torque << std::endl;
    }

    positionSens = newPositionSens;
    pMotor->setFieldVector(0, fabs(torque)); // set motor stator magnetic field vector
}