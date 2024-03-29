/*
 * Commander.h
 *
 *  Created on: 16.10.2020
 *      Author: Marcin
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "mbed.h"
#include "Arbiter.h"
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
    struct Fields       //NOLINT(altera-struct-pack-align)
    {
        uint8_t validData :1;
        uint8_t autopilot :1;
    } fields;
};

class Commander
{
public:
    explicit Commander();
    void displayIncomingReport(const CommandVector& /*cv*/);
    void handler();
    void displaySimData(const CommandVector& /*cv*/) const;
private:
    void connectionOffIndicator() { pcLinkOn = false; connectionLed = GPIO_PIN_RESET; }
    struct SimData      //NOLINT(altera-struct-pack-align)
    {
        uint8_t flapsNumHandlePositions;    // flaps handle positions excluding retracted position
        uint8_t flapsHandleIndex;
        float yokeXreference;       // yoke X reference position received from simulator (w/o vibrations)
        float yokeXposition;        // yoke X axis position sent to simulator
        float yokeZreference;       // yoke Z reference position received from simulator (w/o vibrations)
        float yokeZposition;        // yoke Z axis position sent to simulator        
        SimFlags simFlags;          // bit flags received from simulator
        float normalizedSpeed;      // indicated speed referenced to cruise speed
        float rotationAccBodyX;        // rotation acceleration body X
        float rotationAccBodyY;        // rotation acceleration body Y
        float rotationAccBodyZ;        // rotation acceleration body Z
    };
    void parseReportData();                     // parse received report data
    void sendJoystickData();                    // send joystick data to USB (HID joystick report 1)
    void testJoystickData();                    // fill joystick data for testing
    void sendHidData();                         // send HID buffer data to USB (HID buffer report 2)
    DigitalOut heartBeatLed;                    // Commander heartbeat LED (blue)
    DigitalOut connectionLed;                   // active connection to SimConnect LED (green)
    uint32_t handlerCallCounter{0};             // counter of the handler calls 
    MultiHID PCLink;                            // USB link to PC
    JoystickData joystickData{0};               // structure of joystick data to be sent to PC
    std::vector<uint8_t> receivedReport;        // received report data from PC
    HapticDevice rollDevice;                    // yoke roll axis haptic device
    DigitalIn systemPushbutton;                 // Nucleo board blue button
    SimData simData{0};                         // data received and calculated from simConnect
    Timer sendTimer;                            // measures time between usb reports sending
    Timeout connectionTimeout;                  // USB connection timeout object
    bool pcLinkOn{false};                       // is connection to PC active?
    static constexpr int16_t Max15bit = 0x7FFF; // max 15-bit number
};

#endif /* COMMANDER_H_ */