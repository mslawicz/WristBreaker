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
    MultiPosition,
    Map
};

struct HapticData
{
    float referencePosition;    // reference position of the device
    float torqueGain;           // proportional gain of the torque *see note below
    std::vector<float> detentPositions;     // vector of detent positions
    TorqueMap torqueMap;        // map of the torque function points
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
    MotorBLDC* pMotor;      // BLDC motor
    Encoder* pEncoder;      // motor position encoder
    float positionSens{0};  // motor position read from encoder
    float positionPeriod;   // position segment size of electric 360 degrees cycle
    bool isCalibrated{false};   // true if the device has been calibrated
    float phaseShift{0};    // phase shift between motor electrical phase and sensor phase 
    bool calibrationDirection{true};    // true==up, false==down
    uint8_t calibrationCounter{0};      // counts calibration steps
    float currentPhase{0};
    std::string name;       // the name of this haptic device
    float filteredPosition{0};  // filtered motor position
    static constexpr float FilterRatio = 0.7F;  // 0-no filter
};

#endif /* HAPTIC_H_ */

/*
recommended torqueGain values:
HT2205 Spring: 4
HT2205 MultiPosition: 2-2.5 * number of positions in <0.25-0.75> range
*/