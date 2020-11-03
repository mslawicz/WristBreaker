/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"

HapticDevice::HapticDevice
(
        float positionMin,      // minimal value of motor position
        float positionMax      // maximum value of motor position
) :
    positionMin(positionMin),
    positionMax(positionMax)
{

}