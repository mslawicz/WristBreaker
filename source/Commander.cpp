#include "Commander.h"
#include <iostream>

Commander::Commander(events::EventQueue& eventQueue) :
    eventQueue(eventQueue),
    heartBeatLed(LED1)//,
    //PCLink(USB_PID, USB_VID, USB_VER)
{
    std::cout << "Commander object created\n";
    eventQueue.call_every(HandlerPeriod, this, &Commander::handler);
}


void Commander::handler()
{

    // heart beat
    const uint8_t HeartBeatPattern = 0x50U;
    heartBeatLed = static_cast<int>((handlerCallCounter++ & HeartBeatPattern) == HeartBeatPattern);
}