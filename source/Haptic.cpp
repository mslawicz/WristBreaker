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

//global array for STM Studio tests
float g_value[10];  //XXX test

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    std::string name,       // name of the device
    float referencePosition,    // encoder reference (middle) position of the device
    float maxCalTorque,     // maximum torque value in calibration phase
    float operationRange    // the range of normal operation from reference position
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalTorque(maxCalTorque),
    operationRange(operationRange)
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
    if((state == HapticState::PositionCal) ||
       (state == HapticState::PositionCal))
    {
        state = HapticState::Start;
    }
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
            const float PhaseStepGain = 5.0F * static_cast<float>(pMotor->getNoOfPoles());     // how fast motor should move while finding mid position
            float phaseStep =  PhaseStepGain * error;
            const float PhaseStepLimit = 0.25F * static_cast<float>(pMotor->getNoOfPoles());     // step limit in degrees of electrical revolution
            phaseStep = limit<float>(phaseStep, -PhaseStepLimit, PhaseStepLimit);

            //move motor in the direction of reference position
            currentPhase += phaseStep;

            //ramp of applied torque
            const float TorqueRise = 0.005F;     // 0.5% of torque rise at a time
            torque += maxCalTorque * TorqueRise;
            if(torque > maxCalTorque)
            {
                torque = maxCalTorque;
            }
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabs(currentPosition), PosDevFilterStrength);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                std::cout << "haptic device '" << name << "' ready" << std::endl;
                referencePhase = currentPhase;
                //state = HapticState::HapticAction;
                state = HapticState::HapticAction;
            }

            break;
        }

        //start calibration process
        case HapticState::StartCal:
        {
            break;
        }

        //calibrate a single position
        case HapticState::PositionCal:
        {             
            break;
        }

        //end calibration process
        case HapticState::EndCal:
        {             
            break;
        }        

        //main haptic action
        case HapticState::HapticAction:
        {
            switch(hapticMode)
            {
                //spring action with variable zero position
                case HapticMode::Spring:
                {
                    setTorque(hapticData.zeroPosition, 1.0F);

                    //XXX test
                    static int cnt = 0;
                    if(cnt++ %200 == 0) // NOLINT
                    {
                        std::cout << "pos=" << currentPosition;
                        std::cout << "  pot=" << hapticData.auxData;
                        //std::cout << "  kP=" << kP;
                        //std::cout << "  kD=" << kD;
                        //std::cout << "  P=" << proportional;
                        //std::cout << "  D=" << derivative;
                        std::cout << "  T=" << torque;
                        std::cout << "   \r" << std::flush;
                    }

                    //XXX set global variables
                    g_value[0] = currentPosition;
                    g_value[1] = hapticData.zeroPosition;
                    g_value[3] = hapticData.auxData;
                    g_value[8] = torque;

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

void HapticDevice::setTorque(float zeroPosition, float torqueLimit)
{
    //calculate the current motor electric phase
    currentPhase = cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
    // calculate error from the zero position
    float error = zeroPosition - currentPosition;

    //calculate proportional part of torque
    float kP = 0.2F;    //XXX kP should be provided as argument
    float proportional = kP * error;

    //calculate derivative part of torque
    float kD = 0.95F;  //XXX kD should be provided as argument
    filterEMA<float>(filteredDerivative, error - lastError, 0.2F);
    lastError = error;
    float derivative = kD * filteredDerivative;

    //calculate total requested torque
    torque = proportional + derivative;
    //torque shaping
    torque = (torque > 0 ? 1 : -1) * sqrtf(fabs(torque));
    //torque limit
    torque = limit<float>(torque, -torqueLimit, torqueLimit);

    //apply the requested torque to motor
    float targetPhase = currentPhase + (torque > 0 ? QuarterCycle : -QuarterCycle);
    float torqueMagnitude = fabs(torque);
    pMotor->setFieldVector(targetPhase, torqueMagnitude);

    //XXX test
    g_value[2] = error;
    g_value[4] = kP;
    g_value[5] = kD;
    g_value[6] = proportional;
    g_value[7] = derivative;
}