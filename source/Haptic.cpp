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
    // get the absolute motor shaft position from encoder <0,1>
    encoderPosition = pEncoder->getValue();\
    // calculate shaft position relative to reference position <-0.5...0.5>
    const float EncoderHalfRange = 0.5F;
    float relativePosition = encoderPosition - hapticData.referencePosition;
    if(relativePosition < -EncoderHalfRange)
    {
        relativePosition += 1.0F;
    }
    else if(relativePosition > EncoderHalfRange)
    {
        relativePosition -= 1.0F;
    }
    
    //haptic device state machine
    switch(state)
    {
        // state machine starts here and initializes variables
        case HapticState::Start:
        {
            state = HapticState::Move2Ref;
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            torque = 0.0F;      //motor starts with torque=0 and slowly increase it
            break;
        }

        // motor moves slowly and finds the reference position
        case HapticState::Move2Ref:
        {
            // position error equals -relative position
            float error = -relativePosition;

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
            //move motor in the direction of reference position
            currentPhase += phaseStep;

            //ramp of applied torque
            const float TorqueRise = 0.01F;     // 1% of torque rise at a time
            torque += hapticData.initTorque * TorqueRise;
            if(torque > hapticData.initTorque)
            {
                torque = hapticData.initTorque;
            }
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.98F;
            filterEMA<float>(positionDeviation, fabs(relativePosition), PosDevFilterStrength);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if middle position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                std::cout << "haptic device '" << name << "' ready" << std::endl;
                referencePhase = currentPhase;
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
                //spring action with variable zero position
                case HapticMode::Spring:
                {
                    //calculate the current motor electric phase
                    currentPhase = cropAngle<float>(referencePhase + FullCycle * relativePosition / positionPeriod);
                    // calculate error from the zero position
                    float error = hapticData.zeroPosition - relativePosition;
                    //set torque proportional to the position error
                    torque = scale<float>(-1.0F, 1.0F, hapticData.torqueGain * error, -1.0F, 1.0F);
                    pMotor->setFieldVector(currentPhase + (torque > 0 ? QuarterCycle : -QuarterCycle), fabs(torque));
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
