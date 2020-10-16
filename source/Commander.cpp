#include "Commander.h"
#include <chrono>

Commander::Commander(events::EventQueue& eventQueue) :
    eventQueue(eventQueue),
    heartBeatLed(LED1)
{
    eventQueue.call_every(HandlerPeriod, this, &Commander::handler); // NOLINT(fuchsia-default-arguments-calls)
}


void Commander::handler()
{
    // heart beat
    const uint8_t HeartBeatPattern = 0x50U;
    heartBeatLed = static_cast<int>((handlerCallCounter++ & HeartBeatPattern) == HeartBeatPattern);
}