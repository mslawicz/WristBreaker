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
    float midPosition;          // middle position of the device
    float referencePosition;    // reference position of the device
    float initTorque;           // torque 0..1 applied in the sturtup procedure
    float torqueGain;           // proportional gain of the torque *see note below
    float filterRatio;          // position filter ratio, 0-no filtering
};

class HapticDevice
{
public:
    HapticDevice
    (
        MotorBLDC* pMotor,      // pointer to BLDC motor object
        Encoder* pEncoder,      // pointer to motor position encoder object
        std::string name        // name of the device
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    void calibrationRequest();
    void handler(HapticMode hapticMode, HapticData& hapticData);
private:
    void setTorqueVector(float direction, float magnitude);
    const float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    const float FullCycle = 360.0F;    // full electric cycle in degrees
    MotorBLDC* pMotor;      // BLDC motor
    Encoder* pEncoder;      // motor position encoder
    float encoderPosition{0};  // motor position read from encoder
    float lastRelativePosition{0};   // remembers the last value of encoder relative position 
    float positionPeriod;   // position segment size of electric 360 degrees cycle
    float currentPhase{0};  // current electric phase of the motor
    std::string name;       // the name of this haptic device
    float filteredPosition{0};  // filtered motor position
    enum class HapticState
    {
        Start,
        Move2Mid,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float positionDeviation{0};     //filtered position deviation
};

#endif /* HAPTIC_H_ */

/*
recommended torqueGain values:
HT2205 Spring: 4
*/