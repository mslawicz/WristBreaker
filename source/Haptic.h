/*
 * Haptic.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef HAPTIC_H_
#define HAPTIC_H_

#include <mbed.h>
#include "Convert.h"

class HapticDevice
{
public:
    HapticDevice
    (
        float positionMin,      // minimal value of motor position
        float positionMax      // maximum value of motor position
    );
    float getPositionNorm() const { return scale<float, float>(positionMin, positionMax, positionSens, 0, 1.0F); }
private:
    float positionSens{0};  // motor position read from encoder
    float positionMin;      // minimal value of motor position
    float positionMax;      // maximum value of motor position
};

#endif /* HAPTIC_H_ */