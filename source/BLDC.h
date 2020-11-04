/*
 * BLDC.h
 *
 *  Created on: 04.11.2020
 *      Author: Marcin
 */

#ifndef BLDC_H_
#define BLDC_H_

#include <mbed.h>

class MotorBLDC
{
public:
    MotorBLDC(PinName outA, PinName outB, PinName outC, PinName enable);
private:
    PwmOut phaseA;
    PwmOut phaseB;
    PwmOut phaseC;
    DigitalOut enable;
};

#endif /* BLDC_H_ */