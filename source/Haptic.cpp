/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"

HapticDevice::HapticDevice
(
        Encoder* pEncoder,      // pointer to motor position encoder object
        float positionMin,      // minimal value of motor position
        float positionMax      // maximum value of motor position
) :
    pEncoder(pEncoder),
    positionMin(positionMin),
    positionMax(positionMax)
{

}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
}

// set motor torque vector
// magnitude - torque vector magnitude 0..1 (PWM wave multiplier)
// direction - -1 maximum reverse, 0 hold position, 1 maximum forward
void HapticDevice::setTorqueVector(float  /*magnitude*/, float  /*direction*/)
{
    positionSens = pEncoder->getValue();
}