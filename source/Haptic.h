/*
 * Haptic.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef HAPTIC_H_
#define HAPTIC_H_

#include <mbed.h>
#include "Convert.h"
#include "BLDC.h"
#include "Encoder.h"
#include "Filter.h"
#include <string>


enum class HapticMode
{
    Spring,
    MultiPosition
};

struct HapticData
{
    float targetPosition;       // target position of zero torque (relative to the reference position)
    float torqueGain;           // gain for torque proportional term
    float torqueLimit;          // current maximum torque value
    float feedForward;          // torque feed forward value
    float auxData;              // auxilary data for testing
};

class HapticDevice
{
public:
    HapticDevice
    (
        MotorBLDC* pMotor,      // pointer to BLDC motor object
        Encoder* pEncoder,      // pointer to motor position encoder object
        std::string name,       // name of the device
        float referencePosition,    // encoder reference (middle) position of the device
        float maxCalTorque,      // maximum torque value in calibration phase
        float operationRange,    // the range of normal operation from reference position
        float TD,                //derivative time (see classic PID formula)
        float dTermThreshold,    //threshold for derivative term
        size_t calibrationSections,  //number of calibration sections
        float feedForwardLimit   //limit value of feed forward torque
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    void calibrationRequest();
    void handler(HapticMode hapticMode, HapticData& hapticData);
    float getCurrentPosition() const { return filteredPosition; }     //returns current position of the device relative to reference position
    float getOperationRange() const { return operationRange; }
private:
    float setTorque(HapticData& hapticData);
    const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    const float FullCycle = 360.0F;    // full electric cycle in degrees
    MotorBLDC* pMotor;      // BLDC motor
    Encoder* pEncoder;      // motor position encoder
    float referencePosition;    // encoder reference (middle) position of the device
    float encoderPosition{0};   // motor position read from encoder
    float currentPosition{0};   // current position of the device relative to reference position (not filtered)
    float filteredPosition{0};  // current position of the device relative to reference position (filtered)
    float lastPosition{0};      // last position used for derivative calculations (not filtered)
    float positionPeriod;   // position segment size of electric 360 degrees cycle
    float currentPhase{0};  // current electric phase of the motor
    float referencePhase{0};    // measured electric phase of the motor in the reference position
    std::string name;       // the name of this haptic device
    enum class HapticState
    {
        Start,
        Move2Ref,
        StartCalibration,
        CalibratePosition,
        EndCalibration,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float positionDeviation{0};     //filtered position deviation
    float torque{0.0F};      // current torque of the motor
    float maxCalTorque;      // maximum torque value during calibration phase
    float operationRange;    //the range of normal operation from reference position
    float calibrationPosition{0};
    MedianFilter positionFilter;
    MedianFilter derivativeFilter;
    float TD;        //derivative time (multiplied by torque gain for derivative gain)
    float dTermThreshold;   //threshold for derivative term
    size_t calibrationSections;     //number of calibration sections
    float calibrationTorque{0}; //torque value used in calibration procedure
    float feedForwardLimit;     //limit value of feed forward torque
    size_t counter{0};          //position counter in calibration phase
};

#endif /* HAPTIC_H_ */

/*
recommended motor setup:
57BLY12530 Spring: torqueGain = 1.9 (soft) ... 4.0 (hard); TD=3.9; dTermThreshold=0.015
*/