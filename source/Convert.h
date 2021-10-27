/*
 * Convert.h
 *
 *  Created on: 19.10.2020
 *      Author: Marcin
 */

#ifndef CONVERT_H_
#define CONVERT_H_

#include <cstdint>
#include <cstring>

#define LO8(x)  static_cast<uint8_t>((x)&0xFFU) // NOLINT(hicpp-signed-bitwise)
#define HI8(x)  static_cast<uint8_t>(((x)&0xFF00U)>>8U) // NOLINT(hicpp-signed-bitwise)

//crops float type angle to the range 0...360 degrees
float cropAngle(float angle);

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

template<typename T> void placeData(T data, uint8_t*& pBuffer)
{
    memcpy(pBuffer, &data, sizeof(T));
    pBuffer += sizeof(T);       //NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

template<typename T> T parseData(uint8_t*& pBuffer)
{
    T data;
    memcpy(&data, pBuffer, sizeof(T));
    pBuffer += sizeof(T);       //NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    return data;
}

template<typename T> T limit(T value, T min, T max)
{
    if(value > max)
    {
        value = max;
    }
    else if(value < min)
    {
        value = min;
    }
    return value;
}

//cut out values not exceeding threshold
template<typename T> T threshold(T value, T low, T high)
{
    if(value > high)
    {
        value -= high;
    }
    else if(value < low)
    {
        value -= low;
    }
    else
    {
        value = static_cast<T>(0);
    }
    return value;
}

//check if variable is in the required range
template<typename T> bool isInRange(T value, T min, T max)
{
    return ((value >= min) && (value <= max));
}

float angleDifference(float angle1, float angle2);

template <typename T> unsigned char getParityBit(T data)
{
    const unsigned char BitsInByte = 8; 
    unsigned char bits = BitsInByte * sizeof(T);
    while (bits > 1)
    {
        bits >>= 1;     //NOLINT(hicpp-signed-bitwise)
        data ^= (data >> bits);
    }
    return data & 1;
}

//fast arc tan function
//input: tan<-inf, inf>; output: atan<-90,90>[degrees]
float fastAtan(float tan);

#endif /* CONVERT_H_ */