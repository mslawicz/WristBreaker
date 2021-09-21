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
    return fmod(angle, FullAngle);
}