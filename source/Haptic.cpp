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
    static const float FullCycle = 360.0F;      // full electric cycle in degrees
    static const float CalUpperLimit = 0.7F;    // calibration range upper limit
    static const float CalLowerLimit = 0.3F;    // calibration range lower limit

    // calculate relative electric phase from motor shaft position (shifted from the real position)
    float phaseSens = fmodf(positionSens, positionPeriod) * FullCycle / positionPeriod;

    if(isCalibrated)
    {
        // additional phase shift for generating torque (max 90 degrees)
        direction = scale<float>(-1.0F, 1.0F, direction, -1.0F, 1.0F);
        float targetPhase = phaseSens + phaseShift + direction * QuarterCycle; 
        pMotor->setFieldVector(targetPhase, magnitude);
    }
    else
    {
        // calibration phase
        static const uint8_t NoOfCalibrationSteps = 100;    // number of calibration steps
        static const float CalibrationPhaseStep = 10.0F;    // angle of electric phase movement between steps
        static const float CalibrationVectorMagnitude = 0.6F;   // field vector magnitude during calibration
        if(positionSens > CalUpperLimit)
        {
            calibrationDirection = false;
        }
        else if(positionSens < CalLowerLimit)
        {
            calibrationDirection = true;
        }

        if((positionSens > CalLowerLimit) && (positionSens < CalUpperLimit))
        {
            // motor is within required position range
            if(calibrationCounter != 0) // ignore the first step
            {
                // sum phase shift for all steps
                phaseShift += cropAngle<float>(currentPhase - phaseSens);
            }

            if(calibrationCounter++ == NoOfCalibrationSteps)
            {
                // calibration completed
                isCalibrated = true;
                phaseShift /= NoOfCalibrationSteps; // calculate mean phase shift
                std::cout << name.c_str() << " calibrated (ps=" << phaseShift << ")\n";
            }
        }

        // move motor to next position
        currentPhase = cropAngle<float>(currentPhase + (calibrationDirection ? CalibrationPhaseStep : -CalibrationPhaseStep));
        pMotor->setFieldVector(currentPhase, CalibrationVectorMagnitude);
    }
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    calibrationCounter = 0;
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
            float closestDetentPosition = 0;
            float smallestDistance = 1.0F;
            for(auto& detentPosition : hapticData.detentPositions)
            {
                float distance = fabs(detentPosition - filteredPosition);
                if(distance < smallestDistance)
                {
                    closestDetentPosition = detentPosition;
                    smallestDistance = distance;
                }
            }
            error = closestDetentPosition - filteredPosition;     // distance from closest detent position
            torque = hapticData.torqueGain * error;
        }
        break;

        case HapticMode::Free:
        {
            //static const float FollowRatio = 0.04F;
            float FollowRatio = pot;
            static const float DetentRange = 0.1F;
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
                currentReferencePosition -= error * 0.01F;
            }

            if(filteredPosition < hapticData.minPosition)
            {
                currentReferencePosition = hapticData.minPosition;
            }
            else if(filteredPosition > hapticData.maxPosition)
            {
                currentReferencePosition = hapticData.maxPosition;
            }
            // else if((!hapticData.detentPositions.empty()) &&
            //         (hapticData.detentPositions[0] != 0) &&
            //         (filteredPosition < hapticData.detentPositions[0]) &&
            //         (filteredPosition > hapticData.detentPositions[0] - DetentRange))
            // {
            //     currentReferencePosition = hapticData.detentPositions[0];
            // }

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
                if(fabs(error) > 0.01F)
                {
                    currentReferencePosition -= (error > 0 ? 0.01F : -0.01F);
                }
                gain = 30.0F;
            }
            torque = gain * error;
        }
        break;

        default:
        break;
    }

    setTorqueVector(torque, fabs(torque));

    // static int cnt = 0;
    // if(cnt++ %100 == 0)
    // {
    //     std::cout << "pos=" << positionSens;
    //     std::cout << "  df=" << positionSens - filteredPosition;
    //     std::cout << "  pot=" << pot;
    //     std::cout << "  dir=" << torque;
    //     std::cout << "  mag=" << fabs(torque);
    //     std::cout << std::endl;
    // }
}
