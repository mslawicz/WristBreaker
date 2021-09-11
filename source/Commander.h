/*
 * Commander.h
 *
 *  Created on: 16.10.2020
 *      Author: Marcin
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "mbed.h"
#include "USB.h"
#include "Console.h"
#include "Haptic.h"
#include <chrono>
#include <vector>

struct JoystickData
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

class Commander
{
public:
    explicit Commander(events::EventQueue& eventQueue);
    void displayIncomingReport(const CommandVector& /*cv*/);
private:
    struct SimData
    {
        uint8_t flapsNumHandlePositions;    // flaps handle positions excluding retracted position
        uint8_t flapsHandleIndex;
        uint8_t requestedFlapsHandleIndex;
        bool flapsHandleSetRequest;
        float yokeXreference;       // yoke X reference position received from simulator (w/o vibrations)
        float yokeXposition;        // yoke X axis position sent to simulator
    };
    void handler();
    void parseReportData();                     // parse received report data
    events::EventQueue& eventQueue;             // event queue of the Commander's thread
    DigitalOut heartBeatLed;                    // Commander heartbeat LED
    const std::chrono::milliseconds HandlerPeriod{5};     // period of the handler calls
    uint32_t handlerCallCounter{0};             // counter of the handler calls 
    MultiHID PCLink;                            // USB link to PC
    JoystickData joystickData{0};               // structure of joystick data to be sent to PC
    std::vector<uint8_t> receivedReport;        // received report data from PC
    HapticDevice rollActuator;                  // yoke roll axis actuator
    AnalogIn testPot;                           // XXX test potentiometer
    DigitalIn systemPushbutton;                 // Nucleo board blue button
    SimData simData{0};                         // data received and calculated from simConnect
};

#endif /* COMMANDER_H_ */