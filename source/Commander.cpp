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
    rollActuator
    (
        new MotorBLDC(PD_12, PD_13, PD_14, PE_7, 4),     //NOLINT(readability-magic-numbers)
        new AS5600(PC_4),
        "roll actuator",
        0.75F,                  //NOLINT    device reference position (encoder value)
        0.2F,                   //NOLINT    maximum torque in calibration phase
        0.25F,                  //NOLINT    range of normal operation calculated from reference position
        3.9F,                   //NOLINT    derivative time (see classic PID formula)
        0.015F,                 //NOLINT    threshold for derivative term
        10,                     //NOLINT    number of calibration sections
        0.1F                    //NOLINT    limit value of feed forward torque
    ),
    testPot(PC_5),   //XXX test
    systemPushbutton(BUTTON1)
{
    std::cout << "Commander object created\n";

    // connect USB HID device
    PCLink.connect();

    Console::getInstance().registerCommand("rr", "display latest received report data from PC", callback(this, &Commander::displayIncomingReport));
    Console::getInstance().registerCommand("pec", "program the encoder AS5600 chip", callback(&AS5600::program));
    eventQueue.call_every(HandlerPeriod, this, &Commander::handler);
}


void Commander::handler()
{
    // heart beat
    const uint8_t HeartBeatPattern = 0xA0U;
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

    //calculate pilot's yoke input
    float currentPositionX = rollActuator.getCurrentPosition();     // current poition X of the yoke
    float zeroPositionX = rollActuator.getOperationRange() * simData.yokeXreference;   // requested zero torque position from simulator
    float pilotInputX = currentPositionX - zeroPositionX;

    //XXX test of haptic device
    float pot = testPot.read();
    HapticData& rollActuatorData = rollActuator.getHapticData();
    rollActuatorData.hapticMode = HapticMode::Spring;       //this actuator works in spring mode
    rollActuatorData.goalPosition = 0;   //zeroPositionX,   //zero torque position from simulator
    rollActuatorData.torqueGain = 1.22F;    //NOLINT
    rollActuatorData.deltaPosLimit = 0.001F;    //range 0.5 / 200 Hz / 2.5 sec = 0.001
    rollActuatorData.auxData = pot;

    //XXX test of sinusoidal movement
    //float zeroTest = 0.1F * sin(handlerCallCounter * 0.005F);
    //rollActuatorData.goalPosition = zeroTest;
    //static float fpos = 0.0F;
    //filterEMA<float>(fpos, 0.1F * (pot - 0.5F), 0.95F);
    const float Ampl = 0.1F * pot;
    float fpos = ((handlerCallCounter / 200) & 1) ? Ampl : -Ampl;
    rollActuatorData.goalPosition = fpos;

    rollActuator.handler();

    //prepare data to be sent to simulator 
    // convert +-90 degrees deflection to <-1,1> range
    simData.yokeXposition = scale<float, float>(-rollActuator.getOperationRange(), rollActuator.getOperationRange(), pilotInputX, -1.0F, 1.0F);

    //we do not send joystick reports in this version 
    //PCLink.sendReport(1, joystickReportData);
    //send USB HID report 2
    std::vector<uint8_t> hidData;
    const size_t HidDataSize = 63;
    hidData.resize(HidDataSize);
    uint8_t* pData = hidData.data();
    placeData<float>(simData.yokeXposition , pData);
    placeData<char>('y', pData);
    placeData<char>('o', pData);
    placeData<char>('k', pData);
    placeData<char>('e', pData);
    PCLink.sendReport(2, hidData);

    if(systemPushbutton.read() == 1)
    {
        rollActuator.calibrationRequest();
    }

    //XXX test
    // static int cnt = 0;
    // if(cnt++ %200 == 0) // NOLINT
    // {
    //     std::cout << "posX=" << currentPositionX;
    //     std::cout << "  pot=" << pot;
    //     std::cout << "  zeroX=" << zeroPositionX;
    //     std::cout << "  pilX=" << pilotInputX;
    //     std::cout << "  yokeX=" << simData.yokeXposition;
    //     std::cout << "   \r" << std::flush;
    // }
}

/*
* parse report data received from PC
*/
void Commander::parseReportData()
{
    // if((receivedReport[1] > 0) &&       // flaps handle positions > 0
    //    (receivedReport[2] != simData.flapsHandleIndex) &&    // flaps index has been changed
    //    (receivedReport[2] <= receivedReport[1]))     // handle index <= number of positions excluding position 0
    // {
    //     simData.requestedFlapsHandleIndex = receivedReport[2];
    //     simData.flapsHandleSetRequest = true;
    //     std::cout << "flaps handle position change request " << static_cast<int>(simData.flapsHandleIndex) << "->" << static_cast<int>(receivedReport[2]) << std::endl;
    // }
    uint8_t* pData = receivedReport.data();
    parseData<uint8_t>(pData);      //omit reportID
    simData.flapsNumHandlePositions = parseData<uint8_t>(pData);
    simData.flapsHandleIndex = parseData<uint8_t>(pData);
    simData.yokeXreference = parseData<float>(pData);
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