/*
 * BDC.cpp
 *
 *  Created on: 06.11.2021
 *      Author: Marcin
 */

#include "BDC.h"
#include "Convert.h"

MotorDC::MotorDC(PinName outA, PinName outB) :
    phaseA(outA, 1, true),  //PWM center aligned
    phaseB(outB, 1, true)   //PWM center aligned
{
    static constexpr int PwmPeriodUs = 50;
    this->phaseA.period_us(PwmPeriodUs);
    this->phaseB.period_us(PwmPeriodUs);
}

//set speed of the motor
void MotorDC::setSpeed(float speed)
{
    constexpr float SpeedLimit = 0.5F;  //XXX only for tests with a 6V motor
    constexpr float SpeedShift = 0.18F;     //motor is not spinning below this value
    speed = shift<float>(speed, SpeedShift);
    speed = limit<float>(speed, -SpeedLimit, SpeedLimit);

    phaseA.write(speed > 0 ? speed : 0.0F);
    phaseB.write(speed < 0 ? -speed : 0.0F);
}