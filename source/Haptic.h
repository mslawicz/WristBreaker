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
#include "Console.h"
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
    std::vector<float> targetPositions;     //vector of target positions in multiposition mode
    float targetPosition;       // the current requested target position of zero torque (relative to the reference position)
    float torqueGain;           // gain of the flux vector quadrature component (torque)
    float integralTime;         // integral time (see classic PID formula; TI=1/Ti)
    float directGain;           // gain of the flux vector direct component (damping)
    float magnitudeLimit;       // current maximum magnitude of flux vector
    float deltaPosLimit;        // value of allowed position change; off when ==0
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
        float maxCalMagnitude,   // maximum flux vector magnitude value in calibration phase
        float operationRange,    // the range of normal operation from reference position
        float integralLimit,     //limit of integral term
        uint16_t noOfCalSteps    //number of calibration steps
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
private:
    float setActuator();
    uint8_t getMultipositionIndex();    //gets current position index in multiposition mode
    static constexpr float QuarterCycle = 90.0F;    // 1/4 of electric cycle in degrees
    static constexpr float HalfCycle = 180.0F;    // 1/2 of electric cycle in degrees
    static constexpr float FullCycle = 360.0F;    // full electric cycle in degrees
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
        StartCalibration,
        Calibration,
        Mov2Ref,
        HapticAction
    };
    HapticState state{HapticState::Start};  // state of this haptic device state machine
    float magnitude{0.0F};      // current magnitude of flux vector
    float maxCalMagnitude;      // maximum flux vector magnitude value during calibration phase
    float operationRange;    //the range of normal operation measured from reference position
    MedianFilter positionFilter;    //filters current position
    float targetPosition{0};    //target position used in torque calculations
    HapticData hapticData;      //haptic dynamic parameters of this device
    std::string memParamRefPhase;     //name of parameter in flash memory (referencePhase)
    float iTerm{0};         //integral term of torque
    float integralLimit;      //limit of integral term
    static std::vector<HapticDevice*> hapticDevices;        //NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
    float phaseStep{0};        //phase change during calibration process
    uint16_t counter{0};         //counter for calibration process
    uint16_t noOfCalSteps;      //number of calibration steps
    float vD{0};            //direct component of magnetic flux vector
    float speed{0};         //device speed
    bool isCalibrated{false};   //true if calibrated
    Timer callTimer;        //timer for measuring handler call intervals
    std::chrono::duration<float>::rep interval{0};  //interval time between handler calls
};

#endif /* HAPTIC_H_ */

/*
recommended motor setup:
57BLY12530 Spring: torqueGain = 1.0 (soft) ... 2.8 (hard); TI=7; KD=18;
HT4315 Spring: torqueGain = 5.0; TI=8.5; KD=0;
*/