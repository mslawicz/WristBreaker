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
    float dTermThreshold,    //threshold for derivative term
    size_t calibrationSections,  //number of calibration sections
    float feedForwardLimit   //limit value of feed forward torque
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
    dTermThreshold(dTermThreshold),
    calibrationSections(calibrationSections),
    feedForwardLimit(feedForwardLimit)
{
    pMotor->setEnablePin(1);
    positionPeriod = 2.0F / static_cast<float>(pMotor->getNoOfPoles());
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
    delete pMotor;
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    if(HapticState::HapticAction == state)
    {
        state = HapticState::StartCalibration;
    }
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler()
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

        // start calibration
        case HapticState::StartCalibration:
        {        
            counter = 0;
            calibrationTorque = 0.0F;
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            std::cout << "\nstart of calibration" << std::endl;
            state = HapticState::CalibratePosition;
            break;
        }            

        // calibrates position
        case HapticState::CalibratePosition:
        {
            hapticData.torqueLimit = maxCalTorque;
            hapticData.feedForward = calibrationTorque;
            hapticData.goalPosition = -operationRange + (operationRange + operationRange) * counter / calibrationSections;
            auto error = setTorque();
            const float TorqueStep = 0.01F;     //value of torque increment/decrement
            calibrationTorque = limit<float>(calibrationTorque + error * TorqueStep, -feedForwardLimit, feedForwardLimit);
            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabsf(error), PosDevFilterStrength);
            const float PosDevThreshold = 0.005F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                //store calibration point data here
                //std::cout << "cal=" << hapticData.goalPosition << "  ff = " << hapticData.feedForward << "  err=" << error << "  dev=" << positionDeviation << std::endl;
                std::cout << hapticData.goalPosition << ";" << hapticData.feedForward << std::endl;
                positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
                if(++counter > calibrationSections)
                {
                    state = HapticState::EndCalibration;
                }
            }

            //XXX test
            // static int cnt = 0;
            // if(cnt++ %200 == 0) // NOLINT
            // {
            //     std::cout << "gPos=" << hapticData.goalPosition;
            //     std::cout << "  pos=" << filteredPosition;
            //     std::cout << "  err=" << error;
            //     std::cout << "  tG=" << hapticData.torqueGain;
            //     std::cout << "  dG=" << hapticData.torqueGain * TD;
            //     std::cout << "  T=" << torque;
            //     std::cout << "  ff=" << hapticData.feedForward;
            //     std::cout << "  cPh=" << cropAngle<float>(referencePhase + FullCycle * filteredPosition / positionPeriod);
            //     std::cout << "   \r" << std::flush;
            // }   
            //XXX set global variables
            g_value[0] = filteredPosition;
            g_value[1] = hapticData.goalPosition;
            g_value[8] = torque;                     
            break;
        }

        // end calibration
        case HapticState::EndCalibration:
        {        
            std::cout << "end of calibration" << std::endl;
            state = HapticState::HapticAction;
            break;
        }           

        //main haptic action
        case HapticState::HapticAction:
        {
            switch(hapticData.hapticMode)
            {
                //spring action with variable zero position
                case HapticMode::Spring:
                {
                    static AnalogIn kPpot(PA_5); hapticData.torqueGain = 3.0F * kPpot.read(); //XXX test
                    //static AnalogIn dpPot(PA_6); hapticData.dGain = 10.0F * dpPot.read(); //XXX test
                    //static AnalogIn kDpot(PA_7); float DerivativeThreshold = 0.02F * kDpot.read(); //XXX test 3.3          

                    hapticData.torqueLimit = 1.0F;
                    setTorque();

                    //XXX test
                    static int cnt = 0;
                    if(cnt++ %200 == 0) // NOLINT
                    {
                        std::cout << "pos=" << filteredPosition;
                        std::cout << "  pot=" << hapticData.auxData;
                        std::cout << "  tG=" << hapticData.torqueGain;
                        std::cout << "  dG=" << hapticData.torqueGain * TD;
                        std::cout << "  T=" << torque;
                        std::cout << "  ff=" << hapticData.feedForward;
                        std::cout << "  cPh=" << cropAngle<float>(referencePhase + FullCycle * filteredPosition / positionPeriod);
                        std::cout << "   \r" << std::flush;
                    }

                    //XXX set global variables
                    g_value[0] = filteredPosition;
                    g_value[1] = hapticData.goalPosition;
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

//set torque proportional to goal position error
//returns current error
float HapticDevice::setTorque()
{
    if(hapticData.deltaPosLimit == 0)
    {
        //target position rate of change limit off
        targetPosition = hapticData.goalPosition;
    }
    else
    {
        //limit the rate of change of target position
        targetPosition += limit<float>(hapticData.goalPosition - targetPosition, -hapticData.deltaPosLimit, hapticData.deltaPosLimit);
    }
    //calculate the current motor electric phase
    currentPhase = cropAngle<float>(referencePhase + FullCycle * filteredPosition / positionPeriod);
    //calculate error from the goal position; positive error for CCW deflection
    float error = targetPosition - filteredPosition;
    //calculate proportional term of torque 
    float KP = hapticData.torqueGain;
    float pTerm = KP * error;
    //calculate derivative term of torque
    float deltaPosition = derivativeFilter.getMedian(lastPosition - currentPosition);
    float dTerm = KP * threshold(TD * deltaPosition, -dTermThreshold, dTermThreshold);
    //calculate requested torque with limit
    torque = limit<float>(pTerm + dTerm + hapticData.feedForward, -hapticData.torqueLimit, hapticData.torqueLimit);
    //apply the requested torque to motor
    float deltaPhase = torque > 0 ? QuarterCycle : -QuarterCycle;
    float vectorMagnitude = fabsf(torque);
    pMotor->setFieldVector(currentPhase + deltaPhase, vectorMagnitude);

    lastPosition = currentPosition;

    //XXX test
    g_value[2] = error;
    g_value[3] = deltaPosition;
    g_value[4] = 0;
    g_value[5] = 0;
    g_value[6] = dTerm * 10;
    g_value[7] = pTerm;
    g_value[9] = targetPosition;

    return error;
}