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