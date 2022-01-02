/*
 * ACTUATOR.h
 *
 *  Created on: 26.12.2021
 *      Author: Marcin
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

class Actuator
{
public:
    virtual void enable(bool state) {}
    virtual void setFieldVector(float electricAngle, float magnitude) {}   //XXX temporarily for testing  
    virtual bool calibrate(float encoderPosition) { return true; }     //NOLINT(misc-unused-parameters)
};

#endif /* ACTUATOR_H_ */