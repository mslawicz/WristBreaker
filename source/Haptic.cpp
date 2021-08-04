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
    delete pMotor;
}

// set motor torque vector
// direction: -1 maximum left, 0 hold position, 1 maximum right
// magnitude <0,1>
void HapticDevice::setTorqueVector(float direction, float magnitude)
{
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
    // get motor shaft position from encoder <0,1>
    encoderPosition = pEncoder->getValue();

    //relativePosition is calculated in the range mid-0.5 .. mid+0.5
    const float EncoderMidValue = 0.5F;
    float relativePosition = encoderPosition;
    if(hapticData.midPosition - encoderPosition > EncoderMidValue)
    {
        relativePosition = 1.0F + encoderPosition;
    }
    else if(encoderPosition - hapticData.midPosition > EncoderMidValue)
    {
        relativePosition = encoderPosition - 1.0F;
    }
    
    // filter motor position
    filterEMA<float>(filteredPosition, encoderPosition, hapticData.filterRatio);

    //haptic device state machine
    switch(state)
    {
        // state machine starts here and initializes variables
        case HapticState::Start:
        {
            state = HapticState::Move2Mid;
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            torque = 0.0F;      //motor starts with torque=0
            break;
        }

        // motor moves slowly and finds the middle position
        case HapticState::Move2Mid:
        {
            // calculate error from the middle position
            float error = hapticData.midPosition - relativePosition;

            // calculate motor phase step proportional to error with the limit
            const float PhaseStepGain = 10.0F * static_cast<float>(pMotor->getNoOfPoles());     // how fast motor should move while finding mid position
            float phaseStep =  PhaseStepGain * error;
            const float PhaseStepLimit = 0.5F * static_cast<float>(pMotor->getNoOfPoles());     // step limit in degrees of electrical revolution
            if(phaseStep > PhaseStepLimit)
            {
                phaseStep = PhaseStepLimit;
            }
            else if(phaseStep < -PhaseStepLimit)
            {
                phaseStep = -PhaseStepLimit;
            }

            currentPhase += phaseStep;

            //ramp of applied torque
            const float TorqueRise = 0.01F;     // 1% of torque rise at a time
            torque += hapticData.initTorque * TorqueRise;
            if(torque > hapticData.initTorque)
            {
                torque = hapticData.initTorque;
            }
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation
            const float PosDevFilterStrength = 0.98F;
            filterEMA<float>(positionDeviation, fabs(encoderPosition - hapticData.midPosition), PosDevFilterStrength);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if middle position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                std::cout << "haptic device '" << name << "' ready" << std::endl;
                state = HapticState::HapticAction;
            }

            break;
        }

        //main haptic action
        case HapticState::HapticAction:
        {
            //setTorqueVector(0.0F, 0.0F); 
            switch(hapticMode)
            {
                //spring action with variable reference position
                case HapticMode::Spring:
                {
                    //update the current motor electric phase
                    currentPhase = cropAngle<float>(currentPhase + FullCycle * (relativePosition - lastRelativePosition) / positionPeriod);
                    // calculate error from the reference position
                    float error = hapticData.referencePosition - relativePosition;
                    //set torque proportional to the position error
                    torque = scale<float>(-1.0F, 1.0F, hapticData.torqueGain * error, -1.0F, 1.0F);
                    setTorqueVector(torque > 0 ? 1.0F : -1.0F, 0.5F * hapticData.auxData * fabs(torque));    //NOLINTcppcoreguidelines-avoid-magic-numbers
                    break;
                }

                case HapticMode::MultiPosition:
                default:
                {
                    break;
                }
            }

            break;
        }

        default:
            break;
    }

    lastRelativePosition = relativePosition;

    static int cnt = 0;
    if(cnt++ %100 == 0) // NOLINT
    {
        std::cout << "pos=" << encoderPosition;
        std::cout << "  pot=" << hapticData.auxData;
        std::cout << "  cPh=" << currentPhase;
        std::cout << "  dev=" << positionDeviation;
        std::cout << "  tq=" << torque;
        std::cout << "   \r" << std::flush;
    }
}
