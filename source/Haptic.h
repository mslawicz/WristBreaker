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
#include <map>
#include <vector>

using TorqueVector = std::pair<float, float>;
using TorqueMap = std::map<float, TorqueVector>;

enum class HapticMode
{
    Spring,
    MultiPosition
};

struct HapticData
{
    float zeroPosition;         // position of zero torque (relative to the reference position)
    float torqueGain;           // proportional gain of the torque *see note below
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
        float maxCalTorque,     // maximum torque value in calibration phase
        float operationRange,   // the range of normal operation from reference position
        float kP,               //torque calculation proportional coefficient
        float kD,               //torque calculation derivative coefficient
        size_t derivativeFilterSize,    //size of median filter for derivative part
        float derivativeThreshold   //threshold for derivative part
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    void calibrationRequest();
    void handler(HapticMode hapticMode, HapticData& hapticData);
    void updateMotorPosition();
    float getCurrentPosition() const { return currentPosition; }     //returns current position of the device relative to reference position
    float getOperationRange() const { return operationRange; }
private:
    void setTorque(float zeroPosition, float torqueLimit);
    const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    const float FullCycle = 360.0F;    // full electric cycle in degrees
    MotorBLDC* pMotor;      // BLDC motor
    Encoder* pEncoder;      // motor position encoder
    float referencePosition;    // encoder reference (middle) position of the device
    float encoderPosition{0};  // motor position read from encoder
    float currentPosition{0};  // current position of the device relative to reference position
    float positionPeriod;   // position segment size of electric 360 degrees cycle
    float currentPhase{0};  // current electric phase of the motor
    float referencePhase{0};    // measured electric phase of the motor in the reference position
    std::string name;       // the name of this haptic device
    enum class HapticState
    {
        Start,
        Move2Ref,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float positionDeviation{0};     //filtered position deviation
    float torque{0.0F};     // current torque of the motor
    float maxCalTorque;     // maximum torque value during calibration phase
    float lastPosition{0};     //last position for calculation of derivative component
    float operationRange{0};    //the range of normal operation from reference position
    float calibrationPosition{0};
    float kP{0};            //torque calculation proportional coefficient
    float kD{0};            //torque calculation derivative coefficient
    uint32_t counter{0};    //general counter
    MedianFilter derivativeFilter;  //filter of derivative part
    float derivativeThreshold;  //threshold of torque calculation derivative part
};

#endif /* HAPTIC_H_ */

/*
recommended torqueGain values:
HT2205 Spring: 4
*/