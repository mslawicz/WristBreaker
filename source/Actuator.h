/*
 * ACTUATOR.h
 *
 *  Created on: 26.12.2021
 *      Author: Marcin
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

struct ActuatorData
{
    float encoderValue;
};

class Actuator
{
public:
    virtual void enable(bool state) {}
    virtual void setFieldVector(float electricAngle, float magnitude) {}   //XXX temporarily for testing  
    virtual bool calibrate() { return true; }     //NOLINT(misc-unused-parameters)
    ActuatorData& getActuatorData() { return actuatorData; }
protected:
    ActuatorData actuatorData{0};       //NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,misc-non-private-member-variables-in-classes)
};

#endif /* ACTUATOR_H_ */