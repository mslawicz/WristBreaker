#include "Commander.h"
#include <iostream>

#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5712 //joystick in FS mode +2
#define USB_VER     0x0001 //WristBreaker ver. 1

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

    //XXX test of joystick data
    if(handlerCallCounter & 1) //NOLINT
    {
        int16_t testVal = (handlerCallCounter << 6) & 0xFFFF; // NOLINT
        joystickData.X = testVal;
        joystickData.Y = testVal;
        testVal = (handlerCallCounter << 4) & 0x7FFF; // NOLINT
        joystickData.dial = testVal;
        joystickData.slider = testVal;
        testVal = (handlerCallCounter >> 4) % 9; // NOLINT
        joystickData.hat = testVal;
        testVal = (handlerCallCounter >> 4) & 0xFFFF; // NOLINT
        joystickData.buttons = testVal | (testVal << 16U); // NOLINT
        PCLink.sendReport(joystickData);
    }
    else
    {
        std::vector<uint8_t> testData{1, 2, 3, 4, 5};   // NOLINT
        PCLink.sendReport(testData);
    }

    // send joystick data to PC
    //XXX disabled for test PCLink.sendReport(joystickData);
}