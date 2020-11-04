#include "Commander.h"
#include "Convert.h"
#include "Encoder.h"
#include <iostream>
#include <iomanip>

#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5712 //joystick in FS mode +2
#define USB_VER     0x0001 //WristBreaker ver. 1

Commander::Commander(events::EventQueue& eventQueue) :
    eventQueue(eventQueue),
    heartBeatLed(LED2),
    PCLink(USB_VID, USB_PID, USB_VER),
    throttleLever(new AS5600(PA_3), 0.1F, 0.9F)   //NOLINTreadability-magic-numbers) XXX test
{
    std::cout << "Commander object created\n";

    // connect USB HID device
    PCLink.connect();

    Console::getInstance().registerCommand("rr", "display latest received report data from PC", callback(this, &Commander::displayIncomingReport));
    eventQueue.call_every(HandlerPeriod, this, &Commander::handler);
}


void Commander::handler()
{
    // heart beat
    const uint8_t HeartBeatPattern = 0x50U;
    heartBeatLed = static_cast<int>((handlerCallCounter++ & HeartBeatPattern) == HeartBeatPattern);

    // read USB HID report from PC and parse received simulation data
    while(PCLink.readReport(receivedReport))
    {
        parseReportData();
    }

    //XXX test of joystick data
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

    std::vector<uint8_t> joystickReportData
    {
        LO8(joystickData.X),
        HI8(joystickData.X),
        LO8(joystickData.Y),
        HI8(joystickData.Y),
        LO8(joystickData.Z),
        HI8(joystickData.Z),
        LO8(joystickData.Rz),
        HI8(joystickData.Rz),
        LO8(joystickData.Rx),
        HI8(joystickData.Rx),
        LO8(joystickData.Ry),
        HI8(joystickData.Ry),
        LO8(joystickData.slider),
        HI8(joystickData.slider),
        LO8(joystickData.dial),
        HI8(joystickData.dial),
        joystickData.hat,
        static_cast<uint8_t>(joystickData.buttons & 0xFF),   // NOLINT
        static_cast<uint8_t>((joystickData.buttons >> 8) & 0xFF),   // NOLINT
        static_cast<uint8_t>((joystickData.buttons >> 16) & 0xFF),   // NOLINT
        static_cast<uint8_t>((joystickData.buttons >> 24) & 0xFF)    // NOLINT
    };

    PCLink.sendReport(1, joystickReportData);
    if(handlerCallCounter % 20 == 0) // NOLINT
    {
        std::vector<uint8_t> testData{1, 2, 3, 4, 5};   // NOLINT
        testData.resize(63);    // NOLINT
        PCLink.sendReport(2, testData);
    }
}

/*
* parse report data received from PC
*/
void Commander::parseReportData()   //NOLINT XXX remove when function is done
{
    // XXX test
    std::cout << "i" << std::flush;

}

/*
 * display latest data received from PC
 */
void Commander::displayIncomingReport(const CommandVector& /*cv*/)
{
    const uint8_t ItemsPerLine = 16;
    for(size_t index=0; index < receivedReport.size(); index++)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(receivedReport[index]) << " ";
        if(index % ItemsPerLine == (ItemsPerLine - 1))
        {
            std::cout << std::endl;
        }
    }
}