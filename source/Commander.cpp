#include "Commander.h"
#include "Convert.h"
#include "BLDC.h"
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
    flapsLever   //XXX test
    (
        new MotorBLDC(PA_0, PB_10, PB_11, PE_7, 7),     //NOLINTreadability-magic-numbers)
        new AS5600(PA_6),
        "throttle lever"
    ),
    testPot(PA_5),   //XXX test
    systemPushbutton(BUTTON1)
{
    std::cout << "Commander object created\n";

    // connect USB HID device
    PCLink.connect();

    Console::getInstance().registerCommand("rr", "display latest received report data from PC", callback(this, &Commander::displayIncomingReport));
    Console::getInstance().registerCommand("pec", "program the encoder chip", callback(&Encoder::program));
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

    //XXX test of haptic device
    float pot = testPot.read();
    //HapticData data{.referencePosition = 0.3F + 0.4F * pot, .torqueGain = 4.0F, .filterRatio = 0.8F}; // for Spring
    // HapticData data
    // {
    //     .referencePosition = pot,
    //     .minPosition = 0.2F,
    //     .maxPosition = 0.8F,
    //     .torqueGain = 6.0F,
    //     .filterRatio = 0.7F,
    //     .detentPositions{std::vector<float>{0.3F + 0.4F * pot}}
    // }; // for Free

    // multiposition setup
    HapticData data
    {
        .referencePosition = pot,
        .torqueGain = 6.0F, // NOLINT
        .filterRatio = 0.7F,    // NOLINT
        .setPositionRequest = simData.flapsHandleSetRequest,
        .requestedIndex = simData.requestedFlapsHandleIndex
    }; // for MultiPosition
    // set position detent list
    if(simData.flapsNumHandlePositions < 1)
    {
        data.detentPositions.push_back(0.5F);   // NOLINT the only detent position
    }
    else
    {
        for(uint8_t pI = 0; pI <= simData.flapsNumHandlePositions; pI++)
        {
            data.detentPositions.push_back(0.25F + 0.5F * pI / simData.flapsNumHandlePositions); // NOLINT
        }
    }

    flapsLever.handler(HapticMode::MultiPosition, data);

    // check if requested lever position is achieved
    if((simData.flapsHandleSetRequest) &&
       (flapsLever.getPositionIndex() == simData.flapsHandleIndex))
    {
        // request granted
        simData.flapsHandleSetRequest = false;
    }

    if(systemPushbutton.read() == 1)
    {
        flapsLever.calibrationRequest();
    }

    //we do not send joystick reports in this version PCLink.sendReport(1, joystickReportData);
    std::vector<uint8_t> testData
    {
        flapsLever.getPositionIndex(),
        0x61,   // NOLINT
        0x62,   // NOLINT
        0x63    // NOLINT
    };
    testData.resize(63);    // NOLINT
    PCLink.sendReport(2, testData);
}

/*
* parse report data received from PC
*/
void Commander::parseReportData()
{
    if((receivedReport[1] > 0) &&       // flaps handle positions > 0
       (receivedReport[2] != simData.flapsHandleIndex) &&    // flaps index has been changed
       (receivedReport[2] <= receivedReport[1]))     // handle index <= number of positions excluding position 0
    {
        simData.requestedFlapsHandleIndex = receivedReport[2];
        simData.flapsHandleSetRequest = true;
        std::cout << "flaps handle position change request " << static_cast<int>(simData.flapsHandleIndex) << "->" << static_cast<int>(receivedReport[2]) << std::endl;
    }
    simData.flapsNumHandlePositions = receivedReport[1];
    simData.flapsHandleIndex = receivedReport[2];
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