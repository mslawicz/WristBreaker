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
    state = HapticState::StartCal;
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
            state = HapticState::StartCal;
            break;
        }

        //start calibration process
        case HapticState::StartCal:
        {
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            calibrationPosition = operationRange;
            referencePhase = 0.0F;
            torque = maxCalTorque;
            counter = 0;
            state = HapticState::CalPhase;
            break;
        }

        //measure phase shift
        case HapticState::CalPhase:
        {   
            float error = calibrationPosition - currentPosition;
            currentPhase += limit<float>(5.0F * error, -QuarterCycle, QuarterCycle);
            pMotor->setFieldVector(currentPhase, fabsf(torque));
            //calculate mean position deviation to check if calibration position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabs(currentPosition - lastPosition), PosDevFilterStrength);
            lastPosition = currentPosition;
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                auto measuredRefPhase = cropAngle<float>(currentPhase - FullCycle * currentPosition / positionPeriod);
                counter++;
                std::cout << "pos=" << calibrationPosition;
                std::cout << "  err=" << error;
                std::cout << "  refPh=" << measuredRefPhase; 
                std::cout << std::endl;
                referencePhase += measuredRefPhase;
                const float PositionIncrement = 0.2F * operationRange;
                calibrationPosition -= PositionIncrement;
                positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
                if(calibrationPosition < -operationRange)
                {
                    //end of phase calibration - store the result
                    referencePhase = cropAngle<float>(referencePhase / counter);
                    std::cout << "Mean reference phase=" << referencePhase << std::endl;
                    //store referencePhase here

                    counter = 0;
                    torque = 0.0F;
                    state = HapticState::HapticAction;
                }
            }     
            break;
        }        

        //store calibration position
        case HapticState::CalPosition:
        {   
            const size_t CalibrationSize = 100;
            //calculate target position
            calibrationPosition = -operationRange + counter * (operationRange + operationRange) / CalibrationSize;
            //calculate the current motor electric phase from the motor position
            currentPhase = cropAngle<float>(referencePhase + FullCycle * currentPosition / positionPeriod);
            //calculate error from the zero position; positive error for CCW deflection
            float error = calibrationPosition - currentPosition;
            //integrate requested torque
            torque = limit<float>(3.0F * hapticData.auxData * error, -maxCalTorque, maxCalTorque); 
            float targetPhase = currentPhase + (torque > 0 ? QuarterCycle : -QuarterCycle);
            pMotor->setFieldVector(targetPhase, fabsf(torque));

            //XXX test
            static int cnt = 0;
            if(cnt++ %200 == 0) // NOLINT
            {
                std::cout << "cal=" << calibrationPosition;
                std::cout << "  pos=" << currentPosition;
                std::cout << "  err=" << error;
                std::cout << "  T=" << torque;
                std::cout << "  pot=" << hapticData.auxData;
                std::cout << "   \r" << std::flush;
            }
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

    //calculate derivative part of torque
    static AnalogIn kDpot(PA_7); kD = 3.0F * kDpot.read(); //XXX test
    filterEMA<float>(filteredDerivative, error - lastError, 0.05F);
    lastError = error;
    float derivative = kD * filteredDerivative;

    //calculate total requested torque
    torque = proportional + derivative;
    //torque shaping
    //XXX torque = (torque > 0 ? 1 : -1) * sqrtf(fabs(torque));
    //torque limit
    torque = limit<float>(torque, -torqueLimit, torqueLimit);

    //apply the requested torque to motor
    float targetPhase = currentPhase + (torque > 0 ? QuarterCycle : -QuarterCycle);
    float torqueMagnitude = fabs(torque);
    pMotor->setFieldVector(targetPhase, torqueMagnitude);

    //XXX test
    g_value[2] = error;
    g_value[3] = proportional;
    g_value[4] = 0;
    g_value[5] = derivative;
    g_value[6] = 0;
    g_value[7] = 0;
}