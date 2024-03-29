/*
 * Haptic.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef HAPTIC_H_
#define HAPTIC_H_

#include "Convert.h"
#include "Actuator.h"
#include "Console.h"
#include "Encoder.h"
#include "Filter.h"
#include <algorithm>
#include <mbed.h>
#include <string>
#include <vector>


enum class HapticMode
{
    Spring,
    MultiPosition
};

struct HapticData       //NOLINT(altera-struct-pack-align)
{
    HapticMode hapticMode;      // haptic mode for haptic action phase
    bool useIntegral;           // wether integral term must be used in torque calculations
    bool buttonPressed;         // true if system button pressed
    std::vector<float> targetPositions;     //vector of target positions in multiposition mode
    float targetPosition;       // the current requested target position of zero torque (relative to the reference position)
    float torqueGain;           // gain of the flux vector quadrature component (torque)
    float integralTime;         // integral time (see classic PID formula; TI=1/Ti)
    float directGain;           // gain of the flux vector direct component (damping)
    float magnitudeLimit;       // current maximum magnitude of flux vector
    float deltaPosLimit;        // value of allowed position change; off when ==0
    float positionError;        // error from target position
};

class HapticDevice
{
public:
    HapticDevice        //NOLINT(altera-struct-pack-align)
    (
        Actuator* pActuator,    // pointer to actuator object
        Encoder* pEncoder,      // pointer to position encoder object
        std::string name,       // name of the device
        float referencePosition,    // encoder reference (middle) position of the device
        float operationRange     // the range of normal operation from reference position
    );
    ~HapticDevice();
    HapticDevice(HapticDevice const&) = delete;
    void operator=(HapticDevice const&) = delete;
    HapticDevice(HapticDevice&&) = delete;
    void operator=(HapticDevice&&) = delete;
    static void calibrationRequest(const CommandVector& cv);        //start calibration
    void handler();
    float getCurrentPosition() const { return filteredPosition; }     //returns current position of the device relative to reference position
    float getOperationRange() const { return operationRange; }
    HapticData& getHapticData() { return hapticData; }
    static void listHapticDevices(const CommandVector& cv);        //list all registered haptic devices
    std::string getName() const { return name; }
    void startCalibration() { state = HapticState::StartCalibration; }
    void displayStatus();   //display status of this haptic device
    static void statusRequest(const CommandVector& cv);        //display status
    ActuatorData& getActuatorData() { return pActuator->getActuatorData(); }
private:
    float setActuator();
    uint8_t getMultipositionIndex();    //gets current position index in multiposition mode
    Actuator* pActuator;        // pointer to actuator object
    Encoder* pEncoder;          // actuator position encoder
    float referencePosition;    // encoder reference (middle) position of the device
    float encoderPosition{0};   // actuator position read from encoder
    float currentPosition{0};   // current position of the device relative to reference position (not filtered)
    float filteredPosition{0};  // current position of the device relative to reference position (filtered)
    std::string name;       // the name of this haptic device
    enum class HapticState
    {
        Start,
        StartCalibration,
        Calibration,
        Mov2Ref,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float operationRange;    //the range of normal operation measured from reference position
    MedianFilter positionFilter;    //filters current position
    float targetPosition{0};    //target position used in torque calculations
    HapticData hapticData;      //haptic dynamic parameters of this device
    static std::vector<HapticDevice*> hapticDevices;        //NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    Timer callTimer;        //timer for measuring handler call intervals
    std::chrono::duration<float>::rep interval{0};  //interval time between handler calls
};

#endif /* HAPTIC_H_ */

/*
recommended motor setup:
57BLY12530 Spring: torqueGain = 1.0 (soft) ... 2.8 (hard); TI=7; KD=18;
HT4315 Spring: torqueGain=5.0; TI=8.5; Ilim=0.3; KD=0;
HT4315 Multiposition: torqueGain=6.5; TI=10; Ilim=0.3; KD=0;
HT4315 Spring(throttle): torqueGain=13; KD=0; no integral; ErrorThreshold=0.025; throttleActuatorData.deltaPosLimit=0.002
*/