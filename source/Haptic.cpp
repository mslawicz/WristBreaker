/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include "Convert.h"
#include <cmath>
#include <iostream>
#include <ostream>
#include <utility>

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    std::string name        // name of the device
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name))
{
    pMotor->setEnablePin(1);
    positionPeriod = 1.0F / static_cast<float>(pMotor->getNoOfPoles());
    calibrationRequest();
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
}

// set motor torque vector
// direction: -1 maximum left, 0 hold position, 1 maximum right
// magnitude <0,1>
void HapticDevice::setTorqueVector(float direction, float magnitude)
{
    static const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees

    // additional phase shift for generating torque (max 90 degrees)
    direction = scale<float>(-1.0F, 1.0F, direction, -1.0F, 1.0F);
    float targetPhase = currentPhase + direction * QuarterCycle;
    pMotor->setFieldVector(targetPhase, magnitude);
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    isCalibrated = false;
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler(HapticMode hapticMode, HapticData& hapticData)
{
    float error = 0;        // current position error
    float torque = 0;       // current calculated requested torque

    //temporary solution for clang-tidy(misc-unused-parameters) warning
    if(HapticMode::Spring != hapticMode)
    {
        return;
    }
    
    // get motor shaft position from encoder <0,1>
    encoderPosition = pEncoder->getValue();
    // filter motor position
    filterEMA<float>(filteredPosition, encoderPosition, hapticData.filterRatio);

    float pot = hapticData.referencePosition;   //XXX

    //remaining from spring case
    error = hapticData.referencePosition - filteredPosition;     // error of the current position
    torque = hapticData.torqueGain * error;

    //haptic device state machine
    switch(state)
    {
        case HapticState::Start:
        {
            const float InitialMagnitude = 0.5F;
            currentPhase = 0.0F;
            pMotor->setFieldVector(currentPhase, InitialMagnitude);
            state = HapticState::Phase0;
            break;
        }


        case HapticState::Phase0:
        {
            state = HapticState::Move2Ref;
            break;
        }

        case HapticState::Move2Ref:
        {
            state = HapticState::Move2Ref;
            break;
        }

        default:
            break;
    }


    setTorqueVector(torque, fabs(torque) * 0.7F + 0.3F);    // NOLINT
    //setTorqueVector(pot - 0.5F, 0.0F); //QQQ spinning test

    static int cnt = 0;
    if(cnt++ %100 == 0) // NOLINT
    {
        std::cout << "pos=" << encoderPosition;
        std::cout << "  pot=" << pot;
        // std::cout << "  mag=" << fabs(torque);
        std::cout << "   \r" << std::flush;
    }
}
