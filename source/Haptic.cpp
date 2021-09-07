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
    float maxCalTorque,      // maximum torque value in calibration phase
    float operationRange,    // the range of normal operation from reference position
    float TD,                //derivative time (see classic PID formula)
    float dTermThreshold     //threshold for derivative term
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalTorque(maxCalTorque),
    operationRange(operationRange),
    positionFilter(5),   //NOLINT
    derivativeFilter(5), //NOLINT
    TD(TD),
    dTermThreshold(dTermThreshold)
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
    encoderPosition = pEncoder->getValue();
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
    //filter current position
    filteredPosition = positionFilter.getMedian(currentPosition);

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
            // position error equals -filteredPosition
            float error = -filteredPosition;

            // calculate motor phase step proportional to error with the limit
            const float PhaseStepGain = 2.5F * static_cast<float>(pMotor->getNoOfPoles());     // how fast motor should move while finding mid position
            float phaseStep =  PhaseStepGain * error;
            const float PhaseStepLimit = 0.1F * static_cast<float>(pMotor->getNoOfPoles());     // step limit in degrees of electrical revolution
            phaseStep = limit<float>(phaseStep, -PhaseStepLimit, PhaseStepLimit);

            //move motor in the direction of reference position
            currentPhase += phaseStep;

            //ramp of applied torque
            const float TorqueRise = 0.005F;     // 0.5% of torque rise at a time
            torque += maxCalTorque * TorqueRise;
            torque = limit<float>(torque, 0, maxCalTorque);
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabsf(filteredPosition), PosDevFilterStrength);
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
                    static AnalogIn kPpot(PA_5); hapticData.torqueGain = 3.0F * kPpot.read(); //XXX test
                    //static AnalogIn dpPot(PA_6); hapticData.dGain = 10.0F * dpPot.read(); //XXX test
                    //static AnalogIn kDpot(PA_7); float DerivativeThreshold = 0.02F * kDpot.read(); //XXX test 3.3          

                    setTorque(hapticData.zeroPosition, 1.0F, hapticData);

                    //XXX test
                    static int cnt = 0;
                    if(cnt++ %200 == 0) // NOLINT
                    {
                        std::cout << "pos=" << filteredPosition;
                        std::cout << "  pot=" << hapticData.auxData;
                        std::cout << "  tG=" << hapticData.torqueGain;
                        std::cout << "  dG=" << hapticData.torqueGain * TD;
                        std::cout << "  T=" << torque;
                        std::cout << "  cPh=" << cropAngle<float>(referencePhase + FullCycle * filteredPosition / positionPeriod);
                        std::cout << "   \r" << std::flush;
                    }

                    //XXX set global variables
                    g_value[0] = filteredPosition;
                    g_value[1] = hapticData.zeroPosition;
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

//set torque proportional to target position error
void HapticDevice::setTorque(float targetPosition, float torqueLimit, HapticData& hapticData)
{
    //calculate the current motor electric phase
    currentPhase = cropAngle<float>(referencePhase + FullCycle * filteredPosition / positionPeriod);
    //calculate error from the zero position; positive error for CCW deflection
    float error = targetPosition - filteredPosition;
    //calculate proportional term of torque 
    float pTerm = hapticData.torqueGain * error;
    //calculate derivative term of torque
    float dTerm = TD * derivativeFilter.getMedian(lastPosition - currentPosition);
    dTerm = hapticData.torqueGain * threshold(dTerm, -dTermThreshold, dTermThreshold);
    //calculate requested torque with limit
    torque = limit<float>(pTerm + dTerm, -torqueLimit, torqueLimit);
    //apply the requested torque to motor
    float deltaPhase = torque > 0 ? QuarterCycle : -QuarterCycle;
    float vectorMagnitude = fabsf(torque);
    pMotor->setFieldVector(currentPhase + deltaPhase, vectorMagnitude);

    lastPosition = currentPosition;

    //XXX test
    g_value[2] = error;
    g_value[3] = deltaPhase;
    g_value[4] = 0;
    g_value[5] = 0;
    g_value[6] = dTerm * 10;
    g_value[7] = pTerm;
    g_value[9] = 0;
}