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
#include <chrono>

class Commander
{
public:
    explicit Commander(events::EventQueue& eventQueue);
private:
    void handler();
    events::EventQueue& eventQueue;             // event queue of the Commander's thread
    DigitalOut heartBeatLed;                    // Commander heartbeat LED
    const std::chrono::milliseconds HandlerPeriod{10};     // period of the handler calls
    uint32_t handlerCallCounter{0};             // counter of the handler calls 
    MultiHID PCLink;                            // USB link to PC
    JoystickData joystickData{0};               // structure of joystick data to be sent to PC
};

#endif /* COMMANDER_H_ */