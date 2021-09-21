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
    float TI,                //integral time (see classic PID formula; TI=1/Ti)
    float TD,                //derivative time (see classic PID formula)
    float dThreshold     //threshold for derivative term
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalTorque(maxCalTorque),
    operationRange(operationRange),
    positionFilter(5),   //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    derivativeFilter(5), //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    TI(TI),
    TD(TD),
    dThreshold(dThreshold),
    hapticData{HapticMode::Spring, 0}
{
    pMotor->setEnablePin(1);
    positionPeriod = 2.0F / static_cast<float>(pMotor->getNoOfPoles());     //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
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
        //state = HapticState::StartCalibration;
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
                    state = HapticState::HapticAction;
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
            const float Alpha = 0.015F;
            filterEMA<float>(positionDeviation, fabsf(filteredPosition), Alpha);
            const float PosDevThreshold = 0.01F;    //threshold for stable position
            //check if reference position is reached and stable 
            if(positionDeviation < PosDevThreshold)
            {
                referencePhase = cropAngle(currentPhase);
                std::cout << name << " reference phase measured " << referencePhase << std::endl;
                KvStore::storeData(memParamRefPhase, &referencePhase, sizeof(referencePhase));
                state = HapticState::HapticAction;
            }

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
                    hapticData.torqueLimit = 1.0F;
                    setTorque();
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
//returns current error
float HapticDevice::setTorque()
{
    if(hapticData.deltaPosLimit == 0)
    {
        //target position rate of change limit off
        targetPosition = hapticData.targetPosition;
    }
    else
    {
        //limit the rate of change of target position
        targetPosition += limit<float>(hapticData.targetPosition - targetPosition, -hapticData.deltaPosLimit, hapticData.deltaPosLimit);
    }

    //calculate the current motor electric phase
    currentPhase = cropAngle(referencePhase + FullCycle * filteredPosition / positionPeriod);

    //calculate error from the target position; positive error for CCW deflection
    float error = targetPosition - filteredPosition;

    //calculate proportional term of torque 
    static AnalogIn KPpot(PA_5); hapticData.torqueGain = 2.5F * KPpot.read(); //XXX test
    float KP = hapticData.torqueGain;
    float pTerm = KP * error;

    //calculate integral term of torque
    //static AnalogIn TIpot(PA_6); TI = 0.03F * TIpot.read(); //XXX test 
    iTerm = 0; //limit<float>(iTerm + KP * TI * error, -integralLimit, integralLimit);

    //calculate derivative term of torque
    float deltaPosition = lastPosition - currentPosition;
    float filteredDeltaPosition = derivativeFilter.getMedian(lastPosition - currentPosition);
    static AnalogIn dTpot(PA_6); dThreshold = 0.01F * dTpot.read(); //XXX test 
    auto cutDeltaPosition = threshold<float>(filteredDeltaPosition, -dThreshold, dThreshold);
    static AnalogIn TDpot(PA_7); TD = 10.0F * TDpot.read(); //XXX test
    float dTerm = KP * TD * cutDeltaPosition;
    lastPosition = currentPosition;

    //calculate requested torque with limit
    torque = limit<float>(pTerm + iTerm + dTerm, -hapticData.torqueLimit, hapticData.torqueLimit);

    //apply the requested torque to motor
    float deltaPhase = torque > 0 ? QuarterCycle : -QuarterCycle;
    float vectorMagnitude = fabsf(torque);
    pMotor->setFieldVector(currentPhase + deltaPhase, vectorMagnitude);

    //XXX test
    static int cnt = 0;
    if(cnt++ %200 == 0) // NOLINT
    {
        std::cout << "pos=" << filteredPosition;
        std::cout << "  pot=" << hapticData.auxData;
        std::cout << "  tG=" << hapticData.torqueGain;
        std::cout << "  TI=" << TI;
        std::cout << "  TD=" << TD;
        std::cout << "  dThr=" << dThreshold;
        std::cout << "  T=" << torque;
        std::cout << "  cPh=" << currentPhase;
        std::cout << "   \r" << std::flush;
    }

    //XXX set global variables
    g_value[0] = filteredPosition;
    g_value[1] = hapticData.targetPosition;
    g_value[2] = error;
    g_value[3] = targetPosition;
    g_value[4] = deltaPosition;
    g_value[5] = filteredDeltaPosition;
    g_value[6] = pTerm;
    g_value[7] = iTerm;
    g_value[8] = dTerm;
    g_value[9] = torque;

    return hapticData.targetPosition - filteredPosition;
}