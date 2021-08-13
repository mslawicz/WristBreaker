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
    std::string name,       // name of the device
    float referencePosition,    // encoder reference (middle) position of the device
    float maxCalTorque      // maximum torque value in calibration phase
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalTorque(maxCalTorque)
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

// request calibration process
void HapticDevice::calibrationRequest()
{
    state = HapticState::Start;
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler(HapticMode hapticMode, HapticData& hapticData)
{
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
            // position error equals -currentPosition position
            float error = -currentPosition;

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
            torque += maxCalTorque * TorqueRise;
            if(torque > maxCalTorque)
            {
                torque = maxCalTorque;
            }
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.98F;
            filterEMA<float>(positionDeviation, fabs(currentPosition), PosDevFilterStrength);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if reference position is reached and stable 
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
                    currentPhase = cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
                    // calculate error from the zero position
                    float error = hapticData.zeroPosition - currentPosition;
                    //set torque proportional to the position error
                    torque = scale<float, float>(-1.0F, 1.0F, hapticData.torqueGain * error, -1.0F, 1.0F);

                    //XXX test of derivative part
                    static float lastError = 0.0F;
                    static float filteredDerivative = 0.0F;
                    filterEMA<float>(filteredDerivative, error - lastError, 0.2F);
                    torque += filteredDerivative * 5.0F * hapticData.auxData;
                    lastError = error;

                    //XXX torque shaping test
                    if(fabs(torque) < 0.1F)
                    {
                        torque *= 1.5F;
                    }
                    else
                    {
                        torque = (torque > 0 ? 1.0F : -1.0F) * (0.15F + 0.9444F * (fabs(torque) - 0.1F));
                    }

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
}

void HapticDevice::updateMotorPosition()
{
    encoderPosition = pEncoder->getFilteredValue();
    // calculate shaft position relative to reference position <-0.5...0.5>
    const float EncoderHalfRange = 0.5F;
    float relativePosition = encoderPosition - referencePosition;
    if(relativePosition < -EncoderHalfRange)
    {
        relativePosition += 1.0F;
    }
    else if(relativePosition > EncoderHalfRange)
    {
        relativePosition -= 1.0F;
    }
    currentPosition = relativePosition;
}