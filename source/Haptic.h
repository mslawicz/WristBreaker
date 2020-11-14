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

enum class HapticMode
{
    Spring,
    Free
};

union HapticData
{
    float referencePosition;    // reference position of the device
};

class HapticDevice
{
public:
    HapticDevice
    (
        MotorBLDC* pMotor,      // pointer to BLDC motor object
        Encoder* pEncoder,      // pointer to motor position encoder object
        float positionMin,      // minimal value of motor position
        float positionMax,      // maximum value of motor position
        std::string name,       // name of the device
        float Kp,               // proportional coefficient of the PD controller
        float Kd                // derivative coefficient of the PD controller
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    float getPositionNorm() const { return scale<float, float>(positionMin, positionMax, positionSens, 0, 1.0F); }
    void calibrationRequest();
    void handler(HapticMode hapticMode, HapticData& hapticData);
private:
    void setTorqueVector(float direction, float magnitude);
    MotorBLDC* pMotor;      // BLDC motor
    Encoder* pEncoder;      // motor position encoder
    float positionSens{0};  // motor position read from encoder
    float positionMin;      // minimal value of motor position
    float positionMax;      // maximum value of motor position
    float positionPeriod;   // position segment size of electric 360 degrees cycle
    bool isCalibrated{false};   // true if the device has been calibrated
    float phaseShift{0};    // phase shift between motor electrical phase and sensor phase 
    bool calibrationDirection{true};    // true==up, false==down
    uint8_t calibrationCounter{0};      // counts calibration steps
    float currentPhase{0};
    std::string name;       // the name of this haptic device
    float Kp;               // proportional coefficient of the PD controller
    float Kd;               // derivative coefficient of the PD controller
    float lastError{0};     // last position error used by the PD controller
    float positionNorm{0};  // normalized position of calibration range
};

#endif /* HAPTIC_H_ */