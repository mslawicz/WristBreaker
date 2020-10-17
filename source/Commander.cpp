#include "Commander.h"
#include <iostream>

#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5710 //joystick in FS mode
#define USB_VER     0x0001 //Nucleo Yoke IMU ver. 1

Commander::Commander(events::EventQueue& eventQueue) :
    eventQueue(eventQueue),
    heartBeatLed(LED2),
    PCLink(USB_PID, USB_VID, USB_VER)
{
    std::cout << "Commander object created\n";

    // connect USB HID device
    PCLink.connect();

    eventQueue.call_every(HandlerPeriod, this, &Commander::handler);
}


void Commander::handler()
{

    // heart beat
    const uint8_t HeartBeatPattern = 0x50U;
    heartBeatLed = static_cast<int>((handlerCallCounter++ & HeartBeatPattern) == HeartBeatPattern);
}