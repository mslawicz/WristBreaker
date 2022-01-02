#include "Commander.h"
#include "Stepper.h"
#include "Convert.h"
#include "Encoder.h"
#include "Logger.h"
#include <chrono>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <ratio>


#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5712 //joystick in FS mode +2
#define USB_VER     0x0001 //WristBreaker ver. 1

//global array for STM Studio tests
float g_comm[8];  //XXX test

Commander::Commander() :
    heartBeatLed(LED2),
    connectionLed(LED1),
    PCLink(USB_VID, USB_PID, USB_VER),
    rollDevice
    (
        new Stepper(PE_9, PE_11, PE_13, PE_14, PE_0, 50U),     //NOLINT(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
        new AS5048A(PE_6, PE_5, PE_2, PE_4),
        "roll actuator",
        0.75F,                  //NOLINT    device reference position (encoder value)
        0.25F                   //NOLINT    range of normal operation calculated from reference position
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
    Console::getInstance().registerCommand("dsd", "display received simulator data", callback(this, &Commander::displaySimData));
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
        pcLinkOn = true;
        connectionLed = GPIO_PIN_SET;
        constexpr std::chrono::milliseconds timeout(1200);
        connectionTimeout.attach(callback(this, &Commander::connectionOffIndicator), timeout);
    }

    //calculate pilot's yoke input
    float currentPositionX = rollDevice.getCurrentPosition();     // current poition X of the yoke
    float zeroPositionX = rollDevice.getOperationRange() * limit<float>(simData.yokeXreference, -1.0F, 1.0F);   // requested zero torque position from simulator


    //serve yoke roll actuator
    HapticData& rollDeviceData = rollDevice.getHapticData();
    rollDeviceData.hapticMode = HapticMode::Spring;       //this actuator works in spring mode
    rollDeviceData.useIntegral = (simData.simFlags.fields.autopilot != 0);  //NOLINT(cppcoreguidelines-pro-type-union-access)  use integral when autopilot is on
    rollDeviceData.targetPosition = zeroPositionX;   //zero torque position from simulator <-1,1>
    rollDeviceData.integralTime = 7.0F;        //NOLINT    integral time (see classic PID formula; TI=1/Ti)
    rollDeviceData.deltaPosLimit = 0.0005F;    //range 0.5 / 1000 Hz / 1 sec = 0.0005
    rollDeviceData.magnitudeLimit = 1.0F;      //magnitude limit in action phase
    rollDeviceData.buttonPressed = (systemPushbutton.read() == 1);
    rollDevice.handler();

    //static AnalogIn KPpot(PA_5); yawActuatorData.torqueGain = 10.0F * KPpot.read(); //XXX test; also use PA_6 and PA_7
    //static AnalogIn KLpot(PA_6); yawActuatorData.integralTime = 20.0F * KLpot.read(); //XXX test
    g_comm[2] = simData.rotationAccBodyX; //XXX test 
    g_comm[3] = simData.rotationAccBodyY; //XXX test
    g_comm[4] = simData.rotationAccBodyZ; //XXX test

    //prepare data to be sent to simulator 
    // convert deflection +-operationalRange to <-1,1> range
    simData.yokeXposition = scale<float, float>(-rollDevice.getOperationRange(), rollDevice.getOperationRange(), currentPositionX, -1.0F, 1.0F);
    joystickData.Rz = scale<float, int16_t>(-1.0F, 1.0F, simData.yokeZposition, -Max15bit, Max15bit);

    constexpr auto UsbSendInterval = std::chrono::milliseconds(10);
    if(sendTimer.elapsed_time() >= UsbSendInterval)
    {
        //send USB HID report 1 (HID joystick data)
        sendJoystickData();
        sendTimer.reset();
    }

    // static uint32_t cnt = 0;
    // if(++cnt % 1000 == 0)
    // {
    //     std::cout << currentPositionX;
    //     std::cout << std::endl;
    // }
}

/*
* parse report data received from PC
*/
void Commander::parseReportData()
{
    uint8_t* pData = receivedReport.data();
    parseData<uint8_t>(pData);      //omit reportID
    simData.flapsNumHandlePositions = parseData<uint8_t>(pData);
    simData.flapsHandleIndex = parseData<uint8_t>(pData);
    simData.yokeXreference = parseData<float>(pData);
    simData.simFlags.allFields = parseData<typeof(SimFlags::allFields)>(pData);   //NOLINT(cppcoreguidelines-pro-type-union-access)
    simData.normalizedSpeed = parseData<float>(pData);  // normalized speed
    simData.rotationAccBodyX = parseData<float>(pData);  // rotation acceleration body X
    simData.rotationAccBodyY = parseData<float>(pData);  // rotation acceleration body Y
    simData.rotationAccBodyZ = parseData<float>(pData);  // rotation acceleration body Z
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

/*
 * display deta received and parsed from the simulator
 */
void Commander::displaySimData(const CommandVector& /*cv*/) const
{
    std::cout << "data valid=" << static_cast<int>(simData.simFlags.fields.validData) << std::endl;       //NOLINT(cppcoreguidelines-pro-type-union-access)
    std::cout << "autopilot master=" << static_cast<int>(simData.simFlags.fields.autopilot) << std::endl;       //NOLINT(cppcoreguidelines-pro-type-union-access)
    std::cout << "flaps positions=" << static_cast<int>(simData.flapsNumHandlePositions) << std::endl;
    std::cout << "flaps index=" << static_cast<int>(simData.flapsHandleIndex) << std::endl;
    std::cout << "yoke X ref=" << simData.yokeXreference << std::endl;
}

/*
 * send joystick data to USB (HID joystick report 1)
 */
 void Commander::sendJoystickData()
 {
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
 }

 /*
 * fill joystick data for report data testing
 */
 void Commander::testJoystickData()
 {
    float fx = sin(handlerCallCounter * 0.00628F);  //NOLINT
    int16_t i16 = scale<float, int16_t>(-1.0F, 1.0F, fx, -Max15bit, Max15bit);
    joystickData.X = i16;
    joystickData.Y = i16;
    joystickData.Z = i16;
    joystickData.Rz = i16;
    i16 = scale<float, int16_t>(-1.0F, 1.0F, fx, 0, Max15bit);
    joystickData.Rx = i16;
    joystickData.Ry = i16;
    joystickData.slider = i16;    
    joystickData.dial = i16;
    joystickData.hat = static_cast<uint8_t>((handlerCallCounter >> 6) % 9);  //NOLINT
    joystickData.buttons = static_cast<uint32_t>((handlerCallCounter >> 4) | (handlerCallCounter << 16));  //NOLINT
 }

/*
 * send HID buffer data to USB (HID buffer report 2)
 */
 void Commander::sendHidData()
 {
    std::vector<uint8_t> hidData;
    constexpr size_t HidDataSize = 63;
    hidData.resize(HidDataSize);
    uint8_t* pData = hidData.data();
    placeData<float>(simData.yokeXposition , pData);
    placeData<float>(0.0F, pData);  //reserved for yokeYposition
    placeData<float>(simData.yokeZposition , pData);
    placeData<char>('y', pData);
    placeData<char>('o', pData);
    placeData<char>('k', pData);
    placeData<char>('e', pData);
    PCLink.sendReport(2, hidData);
 }