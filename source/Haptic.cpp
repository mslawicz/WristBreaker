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
#include <cctype>

//global array for STM Studio tests
float g_value[10];  //XXX test

std::vector<HapticDevice*> HapticDevice::hapticDevices;     //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    std::string name,       // name of the device
    float referencePosition,    // encoder reference (middle) position of the device
    float maxCalMagnitude,      // maximum magnitude of flux vector value in calibration phase
    float operationRange,    // the range of normal operation from reference position
    float TI,                //integral time (see classic PID formula; TI=1/Ti)
    float integralLimit,     //limit of integral term
    float KD,                //gain of the direct flux component (for controller stability)
    uint16_t noOfCalSteps    //number of calibration steps
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalMagnitude(maxCalMagnitude),
    operationRange(operationRange),
    positionFilter(5),   //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    TI(TI),
    integralLimit(integralLimit),
    KD(KD),
    noOfCalSteps(noOfCalSteps),
    hapticData{HapticMode::Spring, false, 0}
{
    pMotor->setEnablePin(1);
    positionPeriod = 2.0F / static_cast<float>(pMotor->getNoOfPoles());     //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    hapticDevices.push_back(this);
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
    delete pMotor;
}

// request calibration process
void HapticDevice::calibrationRequest(const CommandVector& cv)
{
    if(cv.size() >= 2)
    {
        if(0 != isdigit(cv[1][0]))
        {
            //the first character of the second argument is a digit
            int deviceIndex = stoi(cv[1]) - 1;
            if(deviceIndex < hapticDevices.size())
            {
                std::cout << "calibrating haptic device " << deviceIndex + 1 << std::endl;
                hapticDevices[deviceIndex]->startCalibration();
            }
            else
            {
                std::cout << "error: haptic device of index " << cv[1] << " not found" << std::endl;
            }
        }
        else
        {
            std::cout << "error: invalid haptic device index " << cv[1] << std::endl;
        }

    }
    else
    {
        std::cout << "error: missing device index" << std::endl;
    }
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler()
{
    const float PhaseStep = operationRange * static_cast<float>(pMotor->getNoOfPoles()) * 0.1F;
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
                    state = HapticState::StartCalibration;
                }
            }
            else
            {
                //parameter not restored
                std::cout << name << " reference phase not restored; calibrating..." << std::endl;
                state = HapticState::StartCalibration;
            }

            positionDeviation = 1.0F;       //ensure the deviation is not close to 0 at start
            break;
        }

        // start calibration process of this device
        case HapticState::StartCalibration:
        {
            std::cout << "device " << name << " calibration started" << std::endl;  //XXX for test only
            currentPhase = 0.0F;
            magnitude = 0.0F;
            phaseStep = PhaseStep;
            counter = 0;
            referencePhase = 0.0F;
            state = HapticState::Calibration;
            break;
        }

        // calibration process
        case HapticState::Calibration:
        {
            const float CalibrationRange = operationRange * 0.7F;
            if(isInRange<float>(filteredPosition, -CalibrationRange, CalibrationRange) &&
                (magnitude >= maxCalMagnitude))
            {
                referencePhase += currentPhase - FullCycle * filteredPosition / positionPeriod;
                counter++;
                g_value[5] = currentPhase - FullCycle * filteredPosition / positionPeriod;
            }
            //change direction of movement if out of calibration range
            if(filteredPosition > CalibrationRange)
            {
                phaseStep = -PhaseStep;
            }
            if(currentPosition < -CalibrationRange)
            {
                phaseStep = PhaseStep;
            }            
            //move motor to next position
            currentPhase += phaseStep;
            //ramp of applied magnitude of flux vector
            const float MagnitudeRise = 0.005F;     // 0.5% of magnitude rise at a time
            magnitude += maxCalMagnitude * MagnitudeRise;
            magnitude = limit<float>(magnitude, 0, maxCalMagnitude);
            //set magnetic flux vector
            pMotor->setFieldVector(currentPhase, magnitude);

            //XXX set global variables
            g_value[0] = filteredPosition;
            g_value[4] = currentPhase;
            g_value[9] = magnitude;            

            if(counter >= noOfCalSteps)
            {
                referencePhase = cropAngle(referencePhase / static_cast<float>(counter));
                std::cout << "device " << name << " has been calibrated with reference phase " << referencePhase << std::endl;
                KvStore::storeData(memParamRefPhase, &referencePhase, sizeof(referencePhase));
                state = HapticState::EndCalibration;
            }
            break;
        }

        // end the calibration process
        case HapticState::EndCalibration:
        {
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
                    hapticData.magnitudeLimit = 1.0F;
                    driver();
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

//drives the motor according to target position error and motor speed
//returns current position error
float HapticDevice::driver()
{
    const float Rad2Deg = 57.2957795F;
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

    //calculate motor speed
    float deltaPosition = lastPosition - currentPosition;
    const float SpeedSmooth = 0.1F;
    filterEMA<float>(speed, deltaPosition, SpeedSmooth);
    lastPosition = currentPosition;

    //calculate proportional term of quadrature component
    float KP = hapticData.torqueGain;
    float pTerm = KP * error;
    //calculate integral term of quadrature component
    if(hapticData.useIntegral)
    {
        iTerm = limit<float>(iTerm + KP * TI * error, -integralLimit, integralLimit);
    }
    else
    {
        iTerm = 0;
    }
    //calculate quadrature component of magnetic flux vector
    float vQ = pTerm + iTerm;   //quadrature component

    //calculate direct component of magnetic flux vector
    float currentVD = KD * fabsf(speed);
    //envelope filter with fast rise and slow decay
    const float RiseFactor = 0.1F;
    const float DecayFactor = 0.005F;
    if(currentVD > vD)
    {
        filterEMA<float>(vD, currentVD, RiseFactor);
    }
    else
    {
        filterEMA<float>(vD, currentVD, DecayFactor);
    }

    //calculate flux vector angle
    float phaseShift{0};
    if(0 == vD)
    {
        phaseShift = vQ > 0 ? QuarterCycle : -QuarterCycle;
    }
    else
    {
        phaseShift = Rad2Deg * atan2f(vQ, vD);
    }
    phaseShift = limit(phaseShift, -QuarterCycle, QuarterCycle);

    //calculate flux vector magnitude
    magnitude = limit<float>(sqrtf(vD * vD + vQ * vQ), 0.0F, hapticData.magnitudeLimit);

    //apply calculated flux vector
    pMotor->setFieldVector(currentPhase + phaseShift, magnitude);

    //XXX test
    static int cnt = 0;
    if(cnt++ %200 == 0) // NOLINT
    {
        std::cout << "pos=" << filteredPosition;
        std::cout << "  pot=" << hapticData.auxData;
        std::cout << "  tG=" << hapticData.torqueGain;
        std::cout << "  KP=" << KP;
        std::cout << "  TI=" << TI;
        std::cout << "  KD=" << KD;
        std::cout << "  magn=" << magnitude;
        std::cout << "  cPh=" << currentPhase;
        std::cout << "   \r" << std::flush;
    }

    //XXX set global variables
    g_value[0] = filteredPosition;
    g_value[1] = hapticData.targetPosition;
    g_value[2] = error;
    g_value[3] = targetPosition;
    g_value[4] = speed * 10000;
    g_value[5] = phaseShift;
    g_value[6] = vQ;
    g_value[7] = vD;
    g_value[8] = magnitude;
    g_value[9] = 0;

    return hapticData.targetPosition - filteredPosition;
}

//list all registered haptic devices
void HapticDevice::listHapticDevices(const CommandVector& /*cv*/)
{
    if(hapticDevices.empty())
    {
        std::cout << "There are no registered haptic devices" << std::endl;
    }
    else
    {
        for(size_t index=0; index < hapticDevices.size(); index++)
        {
            std::cout << "device " << index+1 << ": ";
            std::cout << hapticDevices[index]->getName();
            std::cout << std::endl;
        }
    }
}