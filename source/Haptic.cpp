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
    float operationRange,   // the range of normal operation from reference position
    float kP,               //torque calculation proportional coefficient
    float kD                //torque calculation derivative coefficient    
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalTorque(maxCalTorque),
    operationRange(operationRange),
    kP(kP),
    kD(kD)
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

            //ramp of applied torque
            const float TorqueRise = 0.005F;     // 0.5% of torque rise at a time
            torque += maxCalTorque * TorqueRise;
            torque = limit<float>(torque, 0, maxCalTorque);
            pMotor->setFieldVector(currentPhase, torque);

            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabsf(currentPosition), PosDevFilterStrength);
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
                        std::cout << "  kP=" << kP;
                        std::cout << "  kD=" << kD;
                        std::cout << "  T=" << torque;
                        std::cout << "   \r" << std::flush;
                    }

                    //XXX set global variables
                    g_value[0] = currentPosition;
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

void HapticDevice::setTorque(float targetPosition, float torqueLimit)
{
    //calculate the current motor electric phase
    currentPhase = cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
    // calculate error from the zero position; positive error for CCW deflection
    float error = targetPosition - currentPosition;

    //calculate proportional part of torque
    static AnalogIn kPpot(PA_5); kP = 3.0F * kPpot.read(); //XXX test
    float proportional = kP * error;
    //proportional part shaping
    //proportional = (proportional > 0 ? 1 : -1) * sqrtf(fabsf(proportional));

    //calculate derivative part of torque
    static AnalogIn fltrPot(PA_6); float fltrStr = fltrPot.read(); //XXX test proposed value 0.2F
    static AnalogIn kDpot(PA_7); kD = 10.0F * kDpot.read(); //XXX test
    filterEMA<float>(filteredDerivative, lastPosition - currentPosition, fltrStr);
    lastPosition = currentPosition;
    float derivative = kD * filteredDerivative;
    const float DerivativeThreshold = 0.025;
    if(derivative > DerivativeThreshold)
    {
        derivative -= DerivativeThreshold;
    }
    else if (derivative < -DerivativeThreshold)
    {
        derivative += DerivativeThreshold;
    }
    else
    {
        derivative = 0.0F;
    }

    //calculate total requested torque
    torque = proportional + derivative;
    //torque limit
    torque = limit<float>(torque, -torqueLimit, torqueLimit);

    //apply the requested torque to motor
    float targetPhase = currentPhase + (torque > 0 ? QuarterCycle : -QuarterCycle);
    float torqueMagnitude = fabsf(torque);
    pMotor->setFieldVector(targetPhase, torqueMagnitude);

    //XXX test
    g_value[2] = error;
    g_value[3] = proportional;
    g_value[4] = fltrStr * 0.1F;
    g_value[5] = derivative;
    g_value[6] = 0;
    g_value[7] = 0;
}