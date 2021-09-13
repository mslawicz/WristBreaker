/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include "Convert.h"
#include "Storage.h"
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
    size_t noOfCalPositions, //number of calibration positions
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
    noOfCalPositions(noOfCalPositions),
    feedForwardLimit(feedForwardLimit)
{
    pMotor->setEnablePin(1);
    positionPeriod = 2.0F / static_cast<float>(pMotor->getNoOfPoles());
    Console::getInstance().registerCommand("cal_r", "actuator calibration", callback(this, &HapticDevice::calibrationRequest));
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
    delete pMotor;
}

// request calibration process
void HapticDevice::calibrationRequest(const CommandVector& /*cv*/)
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
            //restore reference phase from flash memory
            memParamRefPhase = "/kv/" + name.substr(0, 3) + '_' + name.substr(name.size()-3, 3) + '_' + "refPhase";
            size_t dataSize = KvStore::getInstance().restoreData(memParamRefPhase, &referencePhase);
            if(sizeof(referencePhase) == dataSize)
            {
                if(isInRange<float>(referencePhase, 0.0F, FullCycle))
                {
                    std::cout << name << " reference phase restored " << referencePhase << std::endl;
                    state = HapticState::FFRestore;
                }
                else
                {
                    std::cout << name << " restored reference phase out of range (" << referencePhase << "); calibrating..." << std::endl;
                    state = HapticState::Move2Ref;
                }
            }
            else
            {
                //parameter not restored
                std::cout << name << " reference phase not restored; calibrating..." << std::endl;
                state = HapticState::Move2Ref;
            }

            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
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
                std::cout << name << " reference phase measured " << referencePhase << std::endl;
                KvStore::getInstance().storeData(memParamRefPhase, &referencePhase, sizeof(referencePhase));
                state = HapticState::FFRestore;
            }

            break;
        }     

        // restore feedforward data
        case HapticState::FFRestore:
        {        
            //restore feed-forward data from flash memory
            feedForwardArray.resize(noOfCalPositions, 0.0F);
            memParamFFData = "/kv/" + name.substr(0, 3) + '_' + name.substr(name.size()-3, 3) + '_' + "FFData";
            size_t dataSize = KvStore::getInstance().restoreData(memParamFFData, feedForwardArray.data());
            if((noOfCalPositions * sizeof(float)) == dataSize)
            {
                std::cout << name << " feed-forward data restored" << std::endl;
            }
            else
            {
                //parameter not restored
                std::cout << name << " feed-forward data not restored; use calibration command" << std::endl;
                feedForwardArray.assign(noOfCalPositions, 0.0F);
                feedForwardArray = std::vector<float>
                {
                    0, 0.05, 0, -0,05,
                    0, 0.05, 0, -0,05,
                    0, 0.05, 0, -0,05,
                    0, 0.05, 0, -0,05,
                    0, 0.05, 0, -0,05,
                    0, 0.05, 0, -0,05,
                    0
                };
            }
            state = HapticState::HapticAction;
            break;
        }    

        // start calibration
        case HapticState::StartCalibration:
        {        
            counter = 0;
            calibrationTorque = 0.0F;
            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            std::cout << name << " calibration started" << std::endl;
            state = HapticState::CalibratePosition;
            break;
        }            

        // calibrates position
        case HapticState::CalibratePosition:
        {
            hapticData.torqueLimit = maxCalTorque;
            hapticData.goalPosition = -operationRange + (operationRange + operationRange) * counter / (noOfCalPositions - 1);
            auto error = setTorque(true);
            const float TorqueStep = 0.01F;     //value of torque increment/decrement
            calibrationTorque = limit<float>(calibrationTorque + error * TorqueStep, -feedForwardLimit, feedForwardLimit);
            //calculate mean position deviation to check if position is reached and stable
            const float PosDevFilterStrength = 0.985F;
            filterEMA<float>(positionDeviation, fabsf(error), PosDevFilterStrength);
            const float PosDevThreshold = 0.005F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                //store feed-forward value
                if(feedForwardArray.size() > counter)
                {
                    feedForwardArray[counter] = calibrationTorque;
                }
                std::cout << "pos=" << hapticData.goalPosition << ", ff=" << calibrationTorque << ", err=" << error << ", dev=" << positionDeviation << std::endl;
                //std::cout << hapticData.goalPosition << ";" << calibrationTorque << std::endl;
                positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
                if(++counter >= noOfCalPositions)
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
            //     std::cout << "  ff=" << calibrationTorque;
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
            std::cout << name << " calibration completed" << std::endl;
            KvStore::getInstance().storeData(memParamFFData, feedForwardArray.data(), noOfCalPositions * sizeof(float));
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
float HapticDevice::setTorque(bool useCalibrationTorque)
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
    //calculate feed-forward term of torque
    float ffTerm{0};
    if(useCalibrationTorque)
    {
        ffTerm = calibrationTorque;
    }
    else
    {
        const float PositionInterval = (operationRange + operationRange) / (noOfCalPositions - 1);
        auto lowerIndex = limit<size_t>(static_cast<size_t>((filteredPosition + operationRange) / PositionInterval), 0, noOfCalPositions - 1);
        float lowerIndexPosition = lowerIndex * PositionInterval - operationRange;
        float positionIntervalPCT = (filteredPosition - lowerIndexPosition) / PositionInterval;
        ffTerm = feedForwardArray[lowerIndex] + (feedForwardArray[lowerIndex+1] - feedForwardArray[lowerIndex]) * positionIntervalPCT;
        static AnalogIn dpPot(PA_6); ffTerm *= dpPot.read(); //XXX test
        g_value[3] = lowerIndex;      
    }
    //calculate requested torque with limit
    torque = limit<float>(pTerm + dTerm + ffTerm, -hapticData.torqueLimit, hapticData.torqueLimit);
    //apply the requested torque to motor
    float deltaPhase = torque > 0 ? QuarterCycle : -QuarterCycle;
    float vectorMagnitude = fabsf(torque);
    pMotor->setFieldVector(currentPhase + deltaPhase, vectorMagnitude);

    lastPosition = currentPosition;

    //XXX test
    g_value[2] = error;
    g_value[4] = 0;
    g_value[5] = 0;
    g_value[6] = dTerm * 10;
    g_value[7] = ffTerm * 10; //pTerm;
    g_value[9] = targetPosition;

    //return hapticData.goalPosition - filteredPosition;
    return error;
}