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
#include <iostream>
#include <ostream>
#include <ratio>
#include <utility>


//global array for STM Studio tests
float g_value[12];  //XXX test

std::vector<HapticDevice*> HapticDevice::hapticDevices;     //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

HapticDevice::HapticDevice
(
    Actuator* pActuator,    // pointer to actuator object
    Encoder* pEncoder,      // pointer to actuator position encoder object
    std::string name,       // name of the device
    float referencePosition,    // encoder reference (middle) position of the device
    float operationRange     // the range of normal operation from reference positions
) :
    pActuator(pActuator),
    pEncoder(pEncoder),
    name(std::move(name)),
    referencePosition(referencePosition),
    operationRange(operationRange),
    positionFilter(5),   //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    hapticData{HapticMode::Spring, false, std::vector<float>(), 0}
{
    pActuator->enable(true);
    hapticDevices.push_back(this);
    callTimer.start();
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
    delete pActuator;
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
            state = HapticState::StartCalibration;
            break;
        }

        // start calibration process of this device
        case HapticState::StartCalibration:
        {
            LOG_DEBUG("device " << name << " calibration started");        
            state = HapticState::Calibration;
            break;
        }

        // calibration process
        case HapticState::Calibration:
        {
            state = HapticState::Mov2Ref;
            break;
        }

        // end the calibration process
        case HapticState::Mov2Ref:
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
                    hapticData.positionError = setActuator();
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

    //calculate error from the target position; positive error for CCW deflection
    float error = targetPosition - filteredPosition;

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

    constexpr uint16_t Sec2Mill = 1000U;
    std::cout << ", interval=" << interval * Sec2Mill << "ms" << std::endl;
    std::cout << "ref pos=" << referencePosition << ", rel pos=" << filteredPosition;
    std::cout << ", tar pos=" << targetPosition << ", oper range=" << -operationRange << "..." << operationRange  << std::endl;
    std::cout << "torque gain=" << hapticData.torqueGain << ", direct gain=" << hapticData.directGain << ", magn limit=" << hapticData.magnitudeLimit;
    std::cout << ", use integral=" << hapticData.useIntegral;
    std::cout << std::endl;
    const std::vector<std::string> ModeName{"spring", "multiposition"};
    std::cout << "mode=" << ModeName[static_cast<size_t>(hapticData.hapticMode)];
    std::cout << std::endl;
    std::cout << "motor poles=" /*<< std::dec << static_cast<int>(pMotor->getNoOfPolePairs()) <<*/ ;
    std::cout << std::endl;
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