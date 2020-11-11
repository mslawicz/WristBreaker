/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include "Convert.h"
#include <cmath>
#include <iostream> //XXX
#include <utility>

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    float positionMin,      // minimal value of motor position
    float positionMax,      // maximum value of motor position
    std::string name        // name of the device
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    positionMin(positionMin),
    positionMax(positionMax),
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

// set motor torque
// torque: -1 maximum reverse, 0 hold position, 1 maximum forward
void HapticDevice::setTorque(float torque)
{
    static const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    static const float FullCycle = 360.0F;      // full electric cycle in degrees
    static const float CalUpperLimit = 0.7F;    // calibration range upper limit
    static const float CalLowerLimit = 0.3F;    // calibration range lower limit

    static int cnt = 0; //XXX test

    // get motor shaft position from encoder (0..1)
    positionSens = pEncoder->getValue();
    // calculate relative electric phase from motor shaft position (shifted from the real position)
    float phaseSens = fmodf(positionSens, positionPeriod) * FullCycle / positionPeriod;

    if(isCalibrated)
    {
        float targetPhase = phaseSens + phaseShift + (torque > 0 ? QuarterCycle : -QuarterCycle); 
        pMotor->setFieldVector(targetPhase, fabs(torque));
    }
    else
    {
        // calibration phase
        static const uint8_t NoOfCalibrationSteps = 100;
        static const float CalibrationPhaseStep = 10.0F;
        static const float CalibrationVectorMagnitude = 0.6F;
        float positionNorm = getPositionNorm();
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
            if(calibrationCounter != 0)
            {
                phaseShift += cropAngle<float>(currentPhase - phaseSens);
            }

            if(calibrationCounter++ == NoOfCalibrationSteps)
            {
                // calibration completed
                isCalibrated = true;
                phaseShift /= NoOfCalibrationSteps;
                std::cout << name.c_str() << " phase shift = " << phaseShift << std::endl;
            }
        }

        currentPhase = cropAngle<float>(currentPhase + (calibrationDirection ? CalibrationPhaseStep : -CalibrationPhaseStep));
        pMotor->setFieldVector(currentPhase, CalibrationVectorMagnitude); // move motor to next position
    }



    if(cnt++ % 127 == 0)
    {
        std::cout << "torque=" << torque << std::endl;
        //std::cout << "   Phase=" << std::endl;
    }
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    calibrationCounter = 0;
    isCalibrated = false;
    phaseShift = 0;
}