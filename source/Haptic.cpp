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
    state = HapticState::Start;
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler(HapticMode hapticMode, HapticData& hapticData)
{
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

    //haptic device state machine
    switch(state)
    {
        // state machine starts here and initializes variables
        case HapticState::Start:
        {
            state = HapticState::Move2Mid;
            break;
        }

        // motor moves slowly and finds the middle position
        case HapticState::Move2Mid:
        {
            const float EncoderMidValue = 0.5F;
            float relativePosition = encoderPosition;
            //relativePosition is calculated in the range mid-0.5 .. mid+0.5
            if(hapticData.midPosition - encoderPosition > EncoderMidValue)
            {
                relativePosition = 1.0F + encoderPosition;
            }
            else if(encoderPosition - hapticData.midPosition > EncoderMidValue)
            {
                relativePosition = encoderPosition - 1.0F;
            }

            // calculate error from the middle position
            float error = hapticData.midPosition - relativePosition;

            // calculate motor phase step proportional to error with the limit
            float PhaseStep = 100.0F * error;
            const float PhaseStepLimit = 10.0F;     // step limit in degrees of electrical revolution
            if(PhaseStep > PhaseStepLimit)
            {
                PhaseStep = PhaseStepLimit;
            }
            else if(PhaseStep < -PhaseStepLimit)
            {
                PhaseStep = -PhaseStepLimit;
            }

            currentPhase += PhaseStep;

            pMotor->setFieldVector(currentPhase, hapticData.initTorque);
            break;
        }

        default:
            break;
    }

    lastEncoderPosition = encoderPosition;

    static int cnt = 0;
    if(cnt++ %100 == 0) // NOLINT
    {
        std::cout << "pos=" << encoderPosition;
        std::cout << "  cPh=" << currentPhase;
        // std::cout << "  mag=" << fabs(torque);
        std::cout << "   \r" << std::flush;
    }
}
