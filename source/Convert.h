/*
 * Convert.h
 *
 *  Created on: 19.10.2020
 *      Author: Marcin
 */

#ifndef CONVERT_H_
#define CONVERT_H_

#define LO8(x)  static_cast<uint8_t>((x)&0xFFU) // NOLINT(hicpp-signed-bitwise)
#define HI8(x)  static_cast<uint8_t>(((x)&0xFF00U)>>8U) // NOLINT(hicpp-signed-bitwise)

template<typename iType, typename oType> oType scale(iType iMin, iType iMax, iType input, oType oMin, oType oMax, bool limit = true)
{
    auto result = static_cast<oType>(1.0F * (input-iMin) / (iMax-iMin) * (oMax-oMin) + oMin);
    if(limit)
    {
        if(result < oMin)
        {
            result = oMin;
        }
        else if(result > oMax)
        {
            result = oMax;
        }
    }
    return result;
}

template<typename Type> Type cropAngle(Type angle)
{
    const Type FullAngle = static_cast<Type>(360);
    while(angle < 0)
    {
        angle += FullAngle;
    };
    while(angle > FullAngle)
    {
        angle -= FullAngle;
    }
    return angle;
}

#endif /* CONVERT_H_ */