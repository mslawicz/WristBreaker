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
    float maxCalForce,      // maximum force value in calibration phase
    float operationRange    // the range of normal operation from reference position  
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalForce(maxCalForce),
    operationRange(operationRange)
{
    pMotor->setEnablePin(1);
    positionPeriod = 2.0F / static_cast<float>(pMotor->getNoOfPoles());
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
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            state = HapticState::Move2Ref;
            break;
        }

        // motor moves slowly and finds the reference position
        case HapticState::Move2Ref:
        {
            // position error equals -currentPosition position
            float error = -currentPosition;

            // calculate motor phase step proportional to error with the limit
            const float PhaseStepGain = 2.5F * static_cast<float>(pMotor->getNoOfPoles());     // how fast motor should move while finding mid position
            float phaseStep =  PhaseStepGain * error;
            const float PhaseStepLimit = 0.1F * static_cast<float>(pMotor->getNoOfPoles());     // step limit in degrees of electrical revolution
            phaseStep = limit<float>(phaseStep, -PhaseStepLimit, PhaseStepLimit);

            //move motor in the direction of reference position
            currentPhase += phaseStep;

            //ramp of applied force
            const float ForceRise = 0.005F;     // 0.5% of force rise at a time
            force += maxCalForce * ForceRise;
            force = limit<float>(force, 0, maxCalForce);
            pMotor->setFieldVector(currentPhase, force);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabsf(currentPosition), PosDevFilterStrength);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                referencePhase = cropAngle<float>(currentPhase);
                std::cout << "haptic device '" << name << "' reference phase = " << referencePhase << std::endl;
                state = HapticState::HapticAction;
            }

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
                    setForce(hapticData.zeroPosition, hapticData.forceGain, 1.0F);

                    //XXX test
                    static int cnt = 0;
                    if(cnt++ %200 == 0) // NOLINT
                    {
                        std::cout << "pos=" << currentPosition;
                        std::cout << "  pot=" << hapticData.auxData;
                        std::cout << "  fG=" << hapticData.forceGain;
                        std::cout << "  F=" << force;
                        std::cout << "  cPh=" << cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
                        std::cout << "   \r" << std::flush;
                    }

                    //XXX set global variables
                    g_value[0] = currentPosition;
                    g_value[1] = hapticData.zeroPosition;
                    g_value[8] = force;

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

//set force vector proportional to target position error
void HapticDevice::setForce(float targetPosition, float forceGain, float forceLimit)
{
    static float lastPosition = 0;

    //calculate the current motor electric phase
    currentPhase = cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
    // calculate error from the zero position; positive error for CCW deflection
    float error = targetPosition - currentPosition;
    //calculate requested force with limit
    static AnalogIn kPpot(PA_5); forceGain = 3.0F * kPpot.read(); //XXX test
    static AnalogIn dpPot(PA_6); float derGain = 10.0F * dpPot.read(); //XXX test
    float derivative = derGain * (lastPosition - currentPosition);
    force = limit<float>(forceGain * error + derivative, -forceLimit, forceLimit);
    //apply the requested force vector to motor
    const float PhaseGain = 2.0F;
    //XXX auto dPhase = limit<float>(PhaseGain * force * QuarterCycle, -QuarterCycle, QuarterCycle);
    float dPhase = force > 0 ? QuarterCycle : -QuarterCycle;
    float vectorMagnitude = fabsf(force);
    pMotor->setFieldVector(currentPhase + dPhase, vectorMagnitude);

    //XXX test
    g_value[2] = error;
    g_value[3] = dPhase;
    g_value[4] = 0;
    g_value[5] = 0;
    g_value[6] = derivative;
    g_value[7] = 0;
    g_value[9] = 0;

    //static AnalogIn kDpot(PA_7); kD = 200.0F * kDpot.read(); //XXX test 3.3
    lastPosition = currentPosition;
}