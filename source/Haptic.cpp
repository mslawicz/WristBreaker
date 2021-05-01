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

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    std::string name        // name of the device
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    name(std::move(name))
{
    pMotor->setEnablePin(1);
    positionPeriod = 1.0F / static_cast<float>(pMotor->getNoOfPoles());
    calibrationRequest();
}

HapticDevice::~HapticDevice()
{
    delete pEncoder;
}

// set motor torque vector
// direction: -1 maximum left, 0 hold position, 1 maximum right
// magnitude <0,1>
void HapticDevice::setTorqueVector(float direction, float magnitude)
{
    static const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees

    // additional phase shift for generating torque (max 90 degrees)
    direction = scale<float>(-1.0F, 1.0F, direction, -1.0F, 1.0F);
    float targetPhase = currentPhase + direction * QuarterCycle;
    pMotor->setFieldVector(targetPhase, magnitude);
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    isCalibrated = false;
    phaseShift = 0;
}

// haptic device application handler
void HapticDevice::handler(HapticMode hapticMode, HapticData& hapticData)
{
    float error = 0;        // current position error
    float torque = 0;       // current calculated requested torque
    
    // get motor shaft position from encoder <0,1>
    positionSens = pEncoder->getValue();
    // filter motor position
    filteredPosition = hapticData.filterRatio * filteredPosition + (1.0F - hapticData.filterRatio) * positionSens;

    float pot = hapticData.referencePosition;   //XXX

    switch(hapticMode)
    {
        case HapticMode::Spring:    // spring with variable reference position
        {
            error = hapticData.referencePosition - filteredPosition;     // error of the current position
            torque = hapticData.torqueGain * error;
        }
        break;

        case HapticMode::MultiPosition:     // multi-position detents
        {
            // calculate current closest position index
            float closestDetentPosition = 0;
            float smallestDistance = 1.0F;
            for(size_t indx = 0; indx < hapticData.detentPositions.size(); indx++)
            {
                float distance = fabs(hapticData.detentPositions[indx] - filteredPosition);
                if(distance < smallestDistance)
                {
                    closestDetentPosition = hapticData.detentPositions[indx];
                    smallestDistance = distance;
                    detentIndex = indx;
                }
            }

            // force position if required by Commander
            if(hapticData.setPositionRequest)
            {
                closestDetentPosition = hapticData.detentPositions[hapticData.requestedIndex];
            }

            error = closestDetentPosition - filteredPosition;     // distance from closest detent position
            torque = hapticData.torqueGain * error;
        }
        break;

        case HapticMode::Free:
        {
            //static const float FollowRatio = 0.04F;
            float FollowRatio = pot;
            static const float DetentRange = 0.05F;
            static const float ErrorThreshold = 0.1F; 
            error = currentReferencePosition - filteredPosition;
            torque = hapticData.torqueGain * error;
            if(fabs(error) > ErrorThreshold)
            {
                currentReferencePosition -= (error > 0 ?
                                            (error - ErrorThreshold) * FollowRatio :
                                            (error + ErrorThreshold) * FollowRatio);
            }
            else
            {
                currentReferencePosition -= error * 0.01F;  // NOLINT
            }

            if(filteredPosition < hapticData.minPosition)
            {
                currentReferencePosition = hapticData.minPosition;
            }
            else if(filteredPosition > hapticData.maxPosition)
            {
                currentReferencePosition = hapticData.maxPosition;
            }
            else if((!hapticData.detentPositions.empty()) &&
                    (hapticData.detentPositions[0] != 0) &&
                    (filteredPosition < hapticData.detentPositions[0] + DetentRange) &&
                    (filteredPosition > hapticData.detentPositions[0] - DetentRange))
            {
                currentReferencePosition = hapticData.detentPositions[0];
            }

        }
        break;

        case HapticMode::Fine:
        {
            error = currentReferencePosition - filteredPosition;
            float gain = hapticData.torqueGain;
            if(filteredPosition < hapticData.minPosition)
            {
                currentReferencePosition = hapticData.minPosition;
            }
            else if(filteredPosition > hapticData.maxPosition)
            {
                currentReferencePosition = hapticData.maxPosition;
            }
            else
            {
                if(fabs(error) > 0.01F)     // NOLINT
                {
                    currentReferencePosition -= (error > 0 ? 0.01F : -0.01F);   // NOLINT
                }
                gain = 30.0F;       // NOLINT
            }
            torque = gain * error;
        }
        break;

        default:
        break;
    }

    setTorqueVector(torque, fabs(torque) * 0.7F + 0.3F);    // NOLINT
    //setTorqueVector(pot - 0.5F, 0.0F); //QQQ spinning test

    static int cnt = 0;
    if(cnt++ %100 == 0) // NOLINT
    {
        std::cout << "pos=" << positionSens;
        std::cout << "  pot=" << pot;
        // std::cout << "  mag=" << fabs(torque);
        std::cout << "   \r" << std::flush;
    }
}
