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
    heartBeatLed.write((handlerCallCounter++ / 50) & 1);    // NOLINT XXX test
}