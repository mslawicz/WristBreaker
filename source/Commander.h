/*
 * Commander.h
 *
 *  Created on: 16.10.2020
 *      Author: Marcin
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_

#include "mbed.h"

class Commander
{
public:
    explicit Commander(const events::EventQueue& eventQueue);
private:
    const events::EventQueue& eventQueue;       // event queue of the Commander's thread
    //DigitalOut heartBeatLed;                  // Commander heartbeat LED
};

#endif /* COMMANDER_H_ */