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
        angle += static_cast<float>(fullTurns + 1) * FullAngle;
    }
    return fmodf(angle, FullAngle);
}

//calculate difference between 2 angles in the range 0-360
float angleDifference(float angle1, float angle2)
{
    const float HalfCycle = 180.0F;    // 1/2 of electric cycle in degrees
    const float FullCycle = 360.0F;    // full electric cycle in degrees
    float dAngle = angle1 - angle2;
    //the difference must be in the range <-180,180>
    if(dAngle > HalfCycle)
    {
        dAngle -= FullCycle;
    } 
    else if(dAngle < -HalfCycle)
    {
        dAngle += FullCycle;
    }
    return dAngle;
}