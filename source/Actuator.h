/*
 * ACTUATOR.h
 *
 *  Created on: 26.12.2021
 *      Author: Marcin
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include <mbed.h>

struct ActuatorData     //NOLINT(altera-struct-pack-align)
{
    float encoderValue; //current encoder value
    float position;     //current position <-0.5,0.5> from reference position
    float calibrationMagnitude; //magniotude during the calibration phase
    float calibrationRange;     //calibration range from reference position 
    uint32_t noOfCalibrationSteps;  //number of calibration steps during the calibration phase
    float calibrationPhaseStep;      //calibration step in electrical degrees 
    float forceGain;            //force gain during action phase
    float minMagnitude;         //minimum magnitude during action phase
    float maxMagnitude;         //maximum magnitude during action phase
};

class Actuator
{
public:
    virtual void enable(bool state) {}
    virtual void calibrationSetup() {}
    virtual bool calibrate() { return true; }
    ActuatorData& getActuatorData() { return actuatorData; }
    virtual void setForce(float error) {} 
protected:
    ActuatorData actuatorData{0};       //NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)  
};

#endif /* ACTUATOR_H_ */