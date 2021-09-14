/*
 * Haptic.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef HAPTIC_H_
#define HAPTIC_H_

#include "Convert.h"
#include "BLDC.h"
#include "Encoder.h"
#include "Filter.h"
#include <mbed.h>
#include <string>


enum class HapticMode
{
    Spring,
    MultiPosition
};

struct HapticData       //NOLINT(altera-struct-pack-align)
{
    HapticMode hapticMode;      // haptic mode for haptic action phase
    float goalPosition;         // goal position of zero torque (relative to the reference position)
    float torqueGain;           // gain for torque proportional term
    float torqueLimit;          // current maximum torque value
    float deltaPosLimit;        // value of allowed position change; off when ==0
    float auxData;              // auxilary data for testing
};

class HapticDevice
{
public:
    HapticDevice        //NOLINT(altera-struct-pack-align)
    (
        MotorBLDC* pMotor,      // pointer to BLDC motor object
        Encoder* pEncoder,      // pointer to motor position encoder object
        std::string name,       // name of the device
        float referencePosition,    // encoder reference (middle) position of the device
        float maxCalTorque,      // maximum torque value in calibration phase
        float operationRange,    // the range of normal operation from reference position
        float TI,                //integral time (see classic PID formula; TI=1/Ti)
        float TD,                //derivative time (see classic PID formula)
        float dTermThreshold     //threshold for derivative term
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    void calibrationRequest(const CommandVector&);
    void handler();
    float getCurrentPosition() const { return filteredPosition; }     //returns current position of the device relative to reference position
    float getOperationRange() const { return operationRange; }
    HapticData& getHapticData() { return hapticData; }
private:
    float setTorque();
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
    float referencePhase{0};    // measured electric phase of the motor in the reference position; stored in flash
    std::string name;       // the name of this haptic device
    enum class HapticState
    {
        Start,
        Move2Ref,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float positionDeviation{0};     //filtered position deviation
    float torque{0.0F};      // current torque of the motor
    float maxCalTorque;      // maximum torque value during calibration phase
    float operationRange;    //the range of normal operation measured from reference position
    MedianFilter positionFilter;
    MedianFilter derivativeFilter;
    float TI;        //integral time (multiplied by torque gain for integral gain)
    float TD;        //derivative time (multiplied by torque gain for derivative gain)
    float dTermThreshold;   //threshold for derivative term
    float targetPosition{0};    //target position used in torque calculations
    HapticData hapticData;      //haptic dynamic parameters of this device
    std::string memParamRefPhase;     //name of parameter in flash memory (referencePhase)
    float iTerm{0};         //integral term of torque
    float integralLimit{0.06F};      //limit of integral term
};

#endif /* HAPTIC_H_ */

/*
recommended motor setup:
57BLY12530 Spring: torqueGain = 0.9 (soft) ... 1.6 (hard); TD=5.6; dTermThreshold=0.03
*/