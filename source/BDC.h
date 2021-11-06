/*
 * BDC.h
 *
 *  Created on: 06.11.2021
 *      Author: Marcin
 */

#ifndef BDC_H_
#define BDC_H_

#include <mbed.h>
#include "FastPWM.h"

class MotorDC
{
public:
    MotorDC(PinName outA, PinName outB);
private:
    FastPWM phaseA;
    FastPWM phaseB;
};

#endif /* BDC_H_ */