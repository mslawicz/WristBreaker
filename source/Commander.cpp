#include "Commander.h"
#include "BLDC.h"
#include "Convert.h"
#include "Encoder.h"
#include "Logger.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <ratio>


#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5712 //joystick in FS mode +2
#define USB_VER     0x0001 //WristBreaker ver. 1

Commander::Commander() :
    heartBeatLed(LED2),
    PCLink(USB_VID, USB_PID, USB_VER),
    rollActuator
    (
        new MotorBLDC(PD_12, PD_13, PD_14, PE_7, 4),     //NOLINT(readability-magic-numbers)
        new AS5600(PC_4),
        "roll actuator",
        0.75F,                  //NOLINT    device reference position (encoder value)
        0.2F,                   //NOLINT    maximum magnitude of flux vector in calibration phase
        0.25F,                  //NOLINT    range of normal operation calculated from reference position
        0.1F,                   //NOLINT    limit of integral term
        500                     //NOLINT    number of calibration steps
    ),
    throttleActuator
    (
        new MotorBLDC(PE_9, PE_11, PE_13, PF_13, 28),     //NOLINT(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
        new AS5048A(PE_6, PE_5, PE_2, PE_4, true),
        "throttle actuator",
        0.75F,                  //NOLINT    device reference position (encoder value)
        0.6F,                   //NOLINT    maximum magnitude of flux vector in calibration phase
        0.25F,                  //NOLINT    range of normal operation calculated from reference position
        0.3F,                   //NOLINT    limit of integral term
        10000                   //NOLINT    number of calibration steps
    ),
    systemPushbutton(BUTTON1)
{
    LOG_INFO("Commander object created");

    // connect USB HID device
    PCLink.connect();
    sendTimer.start();

    Console::getInstance().registerCommand("rr", "display latest received report data from PC", callback(this, &Commander::displayIncomingReport));
    Console::getInstance().registerCommand("pec", "program the encoder AS5600 chip", callback(&AS5600::program));
    Console::getInstance().registerCommand("lhd", "list all registered haptic devices", callback(&HapticDevice::listHapticDevices));
    Console::getInstance().registerCommand("chd", "calibrate haptic device <index>", callback(&HapticDevice::calibrationRequest));
    Console::getInstance().registerCommand("dshd", "display status of haptic device <index>", callback(&HapticDevice::statusRequest));
}

void Commander::handler()
{
    // heart beat
    constexpr uint16_t HeartBeatPattern = 0x05U << 7U;
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
    float zeroPositionX = rollActuator.getOperationRange() * limit<float>(simData.yokeXreference, -1.0F, 1.0F);   // requested zero torque position from simulator
    float pilotInputX = currentPositionX - zeroPositionX;   //pilot's X deflection from zero position

    //serve yoke roll actuator
    HapticData& rollActuatorData = rollActuator.getHapticData();
    rollActuatorData.hapticMode = HapticMode::Spring;       //this actuator works in spring mode
    rollActuatorData.useIntegral = (simData.simFlags.fields.autopilot != 0);  //NOLINT(cppcoreguidelines-pro-type-union-access)  use integral when autopilot is on
    rollActuatorData.targetPosition = zeroPositionX;   //zero torque position from simulator
    rollActuatorData.integralTime = 7.0F;        //NOLINT    integral time (see classic PID formula; TI=1/Ti)
    rollActuatorData.deltaPosLimit = 0.0005F;    //range 0.5 / 1000 Hz / 1 sec = 0.0005
    rollActuatorData.magnitudeLimit = 1.0F;      //magnitude limit in action phase
    //XXX temporarily disabled rollActuator.handler();

    //serve throttle lever
    HapticData& throttleActuatorData = throttleActuator.getHapticData();
    throttleActuatorData.hapticMode = HapticMode::Free;       //this actuator works in free mode
    static AnalogIn KPpot(PA_5); throttleActuatorData.torqueGain = 20.0F * KPpot.read(); //XXX test; also use PA_6 and PA_7
    static AnalogIn KLpot(PA_6); throttleActuatorData.integralTime = 10.0F * KLpot.read(); //XXX test
    static AnalogIn KDpot(PA_7); throttleActuatorData.errorThresholt = 0.05F * KDpot.read(); //XXX test
    throttleActuatorData.useIntegral = (systemPushbutton.read() == 1);
    throttleActuatorData.integralTime = 7.0F;        //NOLINT    integral time (see classic PID formula; TI=1/Ti)
    throttleActuatorData.deltaPosLimit = 0.005F;    //range 0.5 / 1000 Hz / 0.1 sec = 0.005
    throttleActuatorData.magnitudeLimit = 1.0F;      //magnitude limit in action phase
    throttleActuator.handler();    

    //prepare data to be sent to simulator 
    // convert deflection +-operationalRange to <-1,1> range
    simData.yokeXposition = scale<float, float>(-rollActuator.getOperationRange(), rollActuator.getOperationRange(), pilotInputX, -1.0F, 1.0F);
    // convert throttle +-operationalRange to <0,1> range
    constexpr float Half = 0.5F;
    float settledThrottle = Half * (simData.receivedThrottle +
        scale<float, float>(-throttleActuator.getOperationRange(), throttleActuator.getOperationRange(), throttleActuatorData.targetPosition, 0.0F, 1.0F));
    simData.commandedThrottle = limit<float>(settledThrottle, 0.0F, 1.0F);

    //we do not send joystick reports in this version 
    //PCLink.sendReport(1, joystickReportData);
    constexpr auto UsbSendInterval = std::chrono::milliseconds(10);
    if(sendTimer.elapsed_time() > UsbSendInterval)
    {
        //send USB HID report 2
        std::vector<uint8_t> hidData;
        constexpr size_t HidDataSize = 63;
        hidData.resize(HidDataSize);
        uint8_t* pData = hidData.data();
        placeData<float>(simData.yokeXposition , pData);
        placeData<float>(simData.commandedThrottle, pData);
        placeData<char>('y', pData);
        placeData<char>('o', pData);
        placeData<char>('k', pData);
        placeData<char>('e', pData);
        PCLink.sendReport(2, hidData);
        sendTimer.reset();
    }
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
    simData.simFlags.allFields = parseData<typeof(SimFlags::allFields)>(pData);   //NOLINT(cppcoreguidelines-pro-type-union-access)
    simData.receivedThrottle = parseData<float>(pData);
}

/*
 * display latest data received from PC
 */
void Commander::displayIncomingReport(const CommandVector& /*cv*/)
{
    constexpr uint8_t ItemsPerLine = 16;
    for(size_t index=0; index < receivedReport.size(); index++)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(receivedReport[index]) << " ";
        if(index % ItemsPerLine == (ItemsPerLine - 1))
        {
            std::cout << std::endl;
        }
    }
}