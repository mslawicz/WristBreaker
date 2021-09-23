/*
 * Convert.cpp
 *
 *  Created on: 21.09.2021
 *      Author: Marcin
 */

#include "Convert.h"
#include <cmath>

float cropAngle(float angle)
{
    const float FullAngle = 360.0F;
    if(angle < 0)
    {
        int fullTurns = static_cast<int>(-angle / FullAngle);
        angle += static_cast<float>(fullTurns) * FullAngle;
    }
    return fmodf(angle, FullAngle);
}