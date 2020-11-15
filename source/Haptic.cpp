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
    float positionMin,      // minimal value of motor position
    float positionMax,      // maximum value of motor position
    std::string name,       // name of the device
    float Kp,               // proportional coefficient of the PID controller
    float Ki,               // integral coefficient of the PID controller
    float Kd                // derivative coefficient of the PID controller
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    positionMin(positionMin),
    positionMax(positionMax),
    name(std::move(name)),
    Kp(Kp),
    Ki(Ki),
    Kd(Kd)
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

    // get motor shaft position from encoder (0..1)
    positionSens = pEncoder->getValue();
    // calculate relative electric phase from motor shaft position (shifted from the real position)
    float phaseSens = fmodf(positionSens, positionPeriod) * FullCycle / positionPeriod;

    if(isCalibrated)
    {
        // additional 90 degrees phase shift for generating torque
        float targetPhase = phaseSens + phaseShift + direction * QuarterCycle; 
        pMotor->setFieldVector(targetPhase, magnitude);
    }
    else
    {
        // calibration phase
        static const uint8_t NoOfCalibrationSteps = 100;    // number of calibration steps
        static const float CalibrationPhaseStep = 10.0F;    // angle of electric phase movement between steps
        static const float CalibrationVectorMagnitude = 0.6F;   // field vector magnitude during calibration
        if(positionNorm > CalUpperLimit)
        {
            calibrationDirection = false;
        }
        else if(positionNorm < CalLowerLimit)
        {
            calibrationDirection = true;
        }

        if((positionNorm > CalLowerLimit) && (positionNorm < CalUpperLimit))
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
                lastPositionNorm = getPositionNorm();
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
    float direction = 0;    // requested torque vector direction <-1,1>
    float magnitude = 0;    // requested torque vector magnitude <0,1>
    positionNorm = getPositionNorm();     // read normalized position of the device

    switch(hapticMode)
    {
        case HapticMode::Spring:    // spring with variable reference position
        {
            float error = hapticData.referencePosition - positionNorm;     // error of the current position
            float torque = getPID(error);
            direction = torque > 0 ? 1 : -1;    // vector full right or full left
            magnitude = fabs(torque);
        }
        break;

        case HapticMode::Map:      // mapped torque definition
        {
            if(hapticData.torqueMap.empty())
            {
                break;
            }

            float previousPositionMap = 0;
            float previousDirection = 0;
            float previousMagnitude = 0;
            bool isVectorInterpolated = false;

            for(auto const& [positionMap, torqueVectorMap] : hapticData.torqueMap)  //NOLINT
            {
                if(positionNorm <= positionMap)
                {
                    // positionNorm is between this and previous points - interpolate values
                    // calculate relative position in this segment <0,1>
                    float positionInSegment = (positionNorm - previousPositionMap) / (positionMap - previousPositionMap);
                    direction = previousDirection + (torqueVectorMap.first - previousDirection) * positionInSegment;
                    magnitude = previousMagnitude + (torqueVectorMap.second - previousMagnitude) * positionInSegment;
                    isVectorInterpolated = true;
                    break;
                }

                previousPositionMap = positionMap;
                previousDirection = torqueVectorMap.first;
                previousMagnitude = torqueVectorMap.second;
            }

            if(!isVectorInterpolated)
            {
                // positionNorm is greater than the last point in the map - take values of this last point
                direction = previousDirection;
                magnitude = previousMagnitude;
            }

            static int cnt = 0;
            if(cnt++ %100 == 0)
            {
                std::cout << positionNorm << "  d=" << direction << "  m=" << magnitude << std::endl;
            }

        }
        break;

        default:
        break;
    }

    setTorqueVector(direction, magnitude);
}

// get PID controller output
float HapticDevice::getPID(float error)
{
    integralError += error;
    float proportional = Kp * error;
    float integral = Ki * integralError;
    float derivative = Kd * (error - lastError);
    lastError = error;
    return proportional + integral + derivative;
}