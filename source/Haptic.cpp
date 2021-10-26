/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include "Convert.h"
#include "Logger.h"
#include "Storage.h"
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <ostream>
#include <ratio>
#include <utility>


//global array for STM Studio tests
float g_value[12];  //XXX test

std::vector<HapticDevice*> HapticDevice::hapticDevices;     //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    std::string name,       // name of the device
    float referencePosition,    // encoder reference (middle) position of the device
    float maxCalMagnitude,      // maximum magnitude of flux vector value in calibration phase
    float operationRange,    // the range of normal operation from reference position
    float integralLimit,     //limit of integral term
    uint16_t noOfCalSteps    //number of calibration steps
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    maxCalMagnitude(maxCalMagnitude),
    operationRange(operationRange),
    positionFilter(5),   //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    integralLimit(integralLimit),
    noOfCalSteps(noOfCalSteps),
    hapticData{HapticMode::Spring, false, std::vector<float>(), 0}
{
    pMotor->setEnablePin(1);
    constexpr float PolesInPair = 2.0F;
    positionPeriod = PolesInPair / static_cast<float>(pMotor->getNoOfPoles());
    hapticDevices.push_back(this);
    callTimer.start();
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
                LOG_INFO("calibrating haptic device " << deviceIndex + 1);
                hapticDevices[deviceIndex]->startCalibration();
            }
            else
            {
                LOG_ERROR("haptic device of index " << cv[1] << " not found");
            }
        }
        else
        {
                LOG_ERROR("invalid haptic device index " << cv[1]);
        }

    }
    else
    {
        LOG_ERROR("missing device index");
    }
}

// haptic device application handler
// to be called periodically
void HapticDevice::handler()
{
    interval = std::chrono::duration<float>(callTimer.elapsed_time()).count();
    callTimer.reset();
    const float PhaseStep = operationRange * static_cast<float>(pMotor->getNoOfPoles()) * 0.025F;
    encoderPosition = pEncoder->getValue();
    // calculate shaft position relative to reference position <-0.5...0.5>
    constexpr float EncoderHalfRange = 0.5F;
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
                    LOG_INFO(name << " reference phase restored " << referencePhase);
                    isCalibrated = true;
                    targetPosition = currentPosition;      //assures smooth movements to another position
                    state = HapticState::Mov2Ref;
                }
                else
                {
                    LOG_WARNING(name << " restored reference phase out of range (" << referencePhase << "); calibrating...");
                    state = HapticState::StartCalibration;
                }
            }
            else
            {
                //parameter not restored
                LOG_WARNING(name << " reference phase not restored; calibrating...");
                state = HapticState::StartCalibration;
            }

            break;
        }

        // start calibration process of this device
        case HapticState::StartCalibration:
        {
            LOG_DEBUG("device " << name << " calibration started");
            currentPhase = 0.0F;
            magnitude = 0.0F;
            phaseStep = PhaseStep;
            counter = 0;
            referencePhase = 0.0F;
            isCalibrated = false;            
            state = HapticState::Calibration;
            break;
        }

        // calibration process
        case HapticState::Calibration:
        {
            const float CalibrationRange = operationRange * 0.5F;
            if(isInRange<float>(filteredPosition, -CalibrationRange, CalibrationRange) &&
                (magnitude >= maxCalMagnitude))
            {
                referencePhase += currentPhase - FullCycle * filteredPosition / positionPeriod;
                counter++;
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
            const float MagnitudeRise = interval;     // target magnitude will be achieved in 1 second
            magnitude += maxCalMagnitude * MagnitudeRise;
            magnitude = limit<float>(magnitude, 0, maxCalMagnitude);
            //set magnetic flux vector
            pMotor->setFieldVector(currentPhase, magnitude);      

            //XXX test
            g_value[4] = currentPhase;
            g_value[5] = currentPhase - FullCycle * filteredPosition / positionPeriod;
            g_value[8] = magnitude;
            g_value[9] = interval * 100;
            g_value[10] = PhaseStep; //XXX test

            if(counter >= noOfCalSteps)
            {
                referencePhase = cropAngle(referencePhase / static_cast<float>(counter));
                LOG_INFO("device " << name << " has been calibrated with reference phase " << referencePhase);
                KvStore::storeData(memParamRefPhase, &referencePhase, sizeof(referencePhase));
                targetPosition = filteredPosition;      //assures smooth movements to another position
                isCalibrated = true;
                state = HapticState::Mov2Ref;
            }
            break;
        }

        // end the calibration process
        case HapticState::Mov2Ref:
        {
            const float speedLimit = 0.04F * interval;   //slooow!
            hapticData.deltaPosLimit = speedLimit;
            hapticData.magnitudeLimit = maxCalMagnitude;
            hapticData.useIntegral = true;
            float error = setActuator();
            const float AllowedError = operationRange * 0.05F;  //error must be within 5% of operation range
            if(fabsf(error) < AllowedError)
            {
                state = HapticState::HapticAction;
                LOG_DEBUG("device " << name <<  " moved to ref with error=" << error);
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
                    setActuator();
                    break;
                }

                case HapticMode::MultiPosition:
                {
                    uint8_t index = getMultipositionIndex();
                    if(index > hapticData.targetPositions.size())
                    {
                        hapticData.targetPosition = 0.0F;
                    }
                    else
                    {
                        hapticData.targetPosition = hapticData.targetPositions[index];
                    }
                    setActuator();
                    break;
                }

                case HapticMode::Free:
                {
                    float error = setActuator();;
                    auto positionShift = threshold<float>(-error, -hapticData.errorThresholt, hapticData.errorThresholt);
                    hapticData.targetPosition = limit<float>(hapticData.targetPosition + positionShift, -operationRange, operationRange);
                    break;
                }

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
float HapticDevice::setActuator()
{
    constexpr float Rad2Deg = 57.2957795F;
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
    const float SpeedSmooth = 20.0F * interval;
    filterEMA<float>(speed, deltaPosition, SpeedSmooth);
    lastPosition = currentPosition;

    //calculate proportional term of quadrature component
    float KP = hapticData.torqueGain;
    float pTerm = KP * error;
    //calculate integral term of quadrature component
    float TI = hapticData.integralTime;       //integral time (see classic PID formula; TI=1/Ti)
    if(hapticData.useIntegral)
    {
        iTerm = limit<float>(iTerm + KP * TI * error * interval, -integralLimit, integralLimit);
    }
    else
    {
        iTerm = 0;
    }
    //calculate quadrature component of magnetic flux vector
    float vQ = pTerm + iTerm;   //quadrature component

    //calculate direct component of magnetic flux vector
    float KD = hapticData.directGain;       //gain of the flux vector direct component (damping)
    float currentVD =  KD * fabsf(speed);
    //envelope filter with fast rise and slow decay
    const float RiseFactor = 20.0F * interval;
    const float DecayFactor = interval;
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
    // static int cnt = 0;
    // if(cnt++ %200 == 0) // NOLINT
    // {
    //     std::cout << "pos=" << filteredPosition;
    //     std::cout << "  pot=" << hapticData.auxData;
    //     std::cout << "  KP=" << KP;
    //     std::cout << "  TI=" << TI;
    //     std::cout << "  KD=" << KD;
    //     std::cout << "  magn=" << magnitude;
    //     std::cout << "  cPh=" << currentPhase;
    //     std::cout << "   \r" << std::flush;
    // }

    //XXX set global variables
    g_value[0] = filteredPosition;
    g_value[1] = hapticData.targetPosition;
    g_value[2] = error;
    g_value[3] = targetPosition;
    g_value[4] = currentPhase;
    g_value[5] = phaseShift;
    g_value[6] = vQ;
    g_value[7] = vD;
    g_value[8] = magnitude;
    g_value[9] = interval * 100;

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

//display status of haptic device
void HapticDevice::displayStatus()
{
    std::cout << getName() << ", ";
    if(isCalibrated)
    {
        std::cout << "calibrated, ref phase=" << referencePhase;
    }
    else
    {
        std::cout << "not calibrated";
    }
    constexpr uint16_t Sec2Mill = 1000U;
    std::cout << ", interval=" << interval * Sec2Mill << "ms" << std::endl;
    std::cout << "ref pos=" << referencePosition << ", rel pos=" << filteredPosition;
    std::cout << ", tar pos=" << targetPosition << ", oper range=" << -operationRange << "..." << operationRange  << std::endl;
    std::cout << "torque gain=" << hapticData.torqueGain << ", direct gain=" << hapticData.directGain << ", magn limit=" << hapticData.magnitudeLimit;
    std::cout << ", use integral=" << hapticData.useIntegral << ", int time=" << hapticData.integralTime << ", int limit=" << integralLimit << std::endl;
    std::cout << "motor poles=" << std::dec << static_cast<int>(pMotor->getNoOfPoles()) << ", cur phase=" << currentPhase;
    std::cout << ", magn=" << magnitude << std::endl;
    pEncoder->displayStatus();
}

// request status display
void HapticDevice::statusRequest(const CommandVector& cv)
{
    if(cv.size() >= 2)
    {
        if(0 != isdigit(cv[1][0]))
        {
            //the first character of the second argument is a digit
            int deviceIndex = stoi(cv[1]) - 1;
            if(deviceIndex < hapticDevices.size())
            {
                std::cout << "device " << deviceIndex+1 << ": ";
                hapticDevices[deviceIndex]->displayStatus();
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

//gets current position index in multiposition mode
uint8_t HapticDevice::getMultipositionIndex()
{
    constexpr uint8_t NotFound = 0xFF;
    uint8_t shortestIndex{NotFound};
    if(hapticData.targetPositions.empty())
    {
        return shortestIndex;
    }

    uint8_t index{0};
    float shortestDistance{1.0F};
    //find the shortest distance to current position
    do
    {
        float distance = fabsf(hapticData.targetPositions[index] - filteredPosition);
        if(distance < shortestDistance)
        {
            //new shortest distance found
            shortestDistance = distance;
            shortestIndex = index;
        }
        if(distance > shortestDistance)
        {
            //distance bigger than previous one - no need to continue
            break;
        }
    } while(++index < hapticData.targetPositions.size());

    return shortestIndex;
}