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

//fast arc tan function
//input: tan<-inf, inf>; output: atan<-90,90>[degrees]
float fastAtan(float tan)
{
    const float rightAngle = 90.0F;
    float sgn = 1.0F;
    float atan{ 0.0F };
    if (tan < 0)
    {
        tan = -tan;
        sgn = -1.0F;
    }

    if (tan <= 4.0F)    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    {
        atan = (((-0.30269379F * tan + 4.2937098F) * tan - 23.6768389F) * tan + 64.3918945F) * tan;     //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }
    else
    {
        atan = ((((0.00002148232F * tan - 0.001763455F) * tan + 0.057272137F) * tan - 0.93504677F) * tan + 8.036428F) * tan + 55.625009F;      //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    }

    if (atan > rightAngle)
    {
        atan = rightAngle;
    }

    return sgn * atan;
}