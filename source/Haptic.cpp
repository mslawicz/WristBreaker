/*
 * Haptic.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Haptic.h"
#include <iostream> //XXX

HapticDevice::HapticDevice
(
    MotorBLDC* pMotor,      // pointer to BLDC motor object
    Encoder* pEncoder,      // pointer to motor position encoder object
    float positionMin,      // minimal value of motor position
    float positionMax      // maximum value of motor position
) :
    pMotor(pMotor),
    pEncoder(pEncoder),
    positionMin(positionMin),
    positionMax(positionMax)
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

    if(isCalibrated)
    {

    }
    else
    {
        // calibration phase
        static const uint8_t NoOfCalibrationSteps = 100;
        static const float CalibrationPhaseStep = 10.0F;
        static const float CalibrationVectorMagnitude = 0.6F;
        positionSens = pEncoder->getValue();
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
            if(++calibrationCounter == NoOfCalibrationSteps)
            {
                // calibration completed
                isCalibrated = true;
                std::cout << "calibration completed" << std::endl;
            }
        }

        phaseShift += calibrationDirection ? CalibrationPhaseStep : -CalibrationPhaseStep;
        pMotor->setFieldVector(phaseShift, CalibrationVectorMagnitude); // move motor to next position
    }



    if(cnt++ % 127 == 0)
    {
        std::cout << "pos=";
        std::cout << "   Phase=" << std::endl;
    }


    //pMotor->setFieldVector(targetPhase, fabs(torque)); // set motor stator magnetic field vector
}

// request calibration process
void HapticDevice::calibrationRequest()
{
    calibrationCounter = 0;
    isCalibrated = false;
}