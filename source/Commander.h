/*
 * Commander.h
 *
 *  Created on: 16.10.2020
 *      Author: Marcin
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "mbed.h"
#include "Console.h"
#include "Haptic.h"
#include "USB.h"
#include <chrono>
#include <vector>

struct JoystickData     //NOLINT(altera-struct-pack-align)
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t Rx;
    int16_t Ry;
    int16_t Rz;
    int16_t slider;
    int16_t dial;
    uint8_t hat;
    uint32_t buttons;
};

union SimFlags
{
    uint32_t allFields;
    struct Fields
    {
        uint8_t autopilot :1;
    } fields;
};

class Commander
{
public:
    explicit Commander();
    void displayIncomingReport(const CommandVector& /*cv*/);
    void handler();
private:
    struct SimData      //NOLINT(altera-struct-pack-align)
    {
        uint8_t flapsNumHandlePositions;    // flaps handle positions excluding retracted position
        uint8_t flapsHandleIndex;
        uint8_t requestedFlapsHandleIndex;
        bool flapsHandleSetRequest;
        float yokeXreference;       // yoke X reference position received from simulator (w/o vibrations)
        float yokeXposition;        // yoke X axis position sent to simulator
        SimFlags simFlags;          // bit flags received from simulator
    };
    void parseReportData();                     // parse received report data
    DigitalOut heartBeatLed;                    // Commander heartbeat LED
    const std::chrono::milliseconds HandlerPeriod{5};     // period of the handler calls
    uint32_t handlerCallCounter{0};             // counter of the handler calls 
    MultiHID PCLink;                            // USB link to PC
    JoystickData joystickData{0};               // structure of joystick data to be sent to PC
    std::vector<uint8_t> receivedReport;        // received report data from PC
    HapticDevice rollActuator;                  // yoke roll axis actuator
    HapticDevice throttleActuator;              // throttle lever actuator
    DigitalIn systemPushbutton;                 // Nucleo board blue button
    SimData simData{0};                         // data received and calculated from simConnect
};

#endif /* COMMANDER_H_ */