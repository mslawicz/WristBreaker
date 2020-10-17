/*
 * USB.cpp
 *
 *  Created on: 17.10.2020
 *      Author: Marcin
 */

#include "USB.h"
#include "usb_phy_api.h"
#include <iostream>
#include <iomanip>
#include <vector>

#define LOW(n) *reinterpret_cast<uint8_t*>(&n)
#define HIGH(n) *reinterpret_cast<uint8_t*>(&n+1)   /*NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)*/

MultiHID::MultiHID(uint16_t vendorId, uint16_t productId, uint16_t productRelease, bool blocking) :
    USBHID(get_usb_phy(), 0, 0, vendorId, productId, productRelease)
{
    if (blocking)
    {
        std::cout << std::hex << setfill('0') << setw(4) << "Connecting USB HID joystick device (VID=0x" << vendorId << ", PID=0x" << productId;
        std::cout << std::dec << ", VER=" << productRelease << ") in non-blocking mode\n";
        USBDevice::connect();
        wait_ready();
    }
    else
    {
        std::cout << std::hex << setfill('0') << setw(4) << "Initializing USB HID joystick device (VID=0x" << vendorId << ", PID=0x" << productId;
        std::cout << std::dec << ", VER=" << productRelease << ") in blocking mode\n";
        init();
    }
}

MultiHID::~MultiHID() noexcept
{
    deinit();
}

const uint8_t* MultiHID::report_desc()
{
    static const std::vector<uint8_t> reportDescriptor
    {
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x04,                    // USAGE (Joystick)
        0xa1, 0x01,                    // COLLECTION (Application)
        0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
        0x09, 0x01,                    //   USAGE (Pointer)
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x75, 0x10,                    //     REPORT_SIZE (16)
        0x16, 0x01, 0x80,              //     LOGICAL_MINIMUM (-32767)
        0x26, 0xff, 0x7f,              //     LOGICAL_MAXIMUM (32767)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x09, 0x32,                    //     USAGE (Z)
        0x09, 0x35,                    //     USAGE (Rz)
        0x95, 0x04,                    //     REPORT_COUNT (4)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x7f,              //     LOGICAL_MAXIMUM (32767)
        0x09, 0x33,                    //     USAGE (Rx)
        0x09, 0x34,                    //     USAGE (Ry)
        0x09, 0x36,                    //     USAGE (slider)
        0x09, 0x37,                    //     USAGE (dial)
        0x95, 0x04,                    //     REPORT_COUNT (4)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          //   END_COLLECTION
        0x09, 0x39,                    //   USAGE (Hat switch)
        0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
        0x25, 0x08,                    //   LOGICAL_MAXIMUM (8)
        0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
        0x46, 0x3b, 0x01,              //   PHYSICAL_MAXIMUM (315)
        0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
        0x75, 0x04,                    //   REPORT_SIZE (4)
        0x95, 0x01,                    //   REPORT_COUNT (1)
        0x81, 0x42,                    //   INPUT (Data,Var,Abs, Null)
        0x75, 0x04,                    //   REPORT_SIZE (4)
        0x95, 0x01,                    //   REPORT_COUNT (1)
        0x81, 0x41,                    //   INPUT (Cnst,Ary,Abs,Null)
        0x05, 0x09,                    //   USAGE_PAGE (Button)
        0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
        0x29, 0x20,                    //   USAGE_MAXIMUM (Button 32)
        0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
        0x75, 0x01,                    //   REPORT_SIZE (1)
        0x95, 0x20,                    //   REPORT_COUNT (16)
        0x55, 0x00,                    //   UNIT_EXPONENT (0)
        0x65, 0x00,                    //   UNIT (None)
        0x81, 0x02,                    //   INPUT (Data,Var,Abs)
        0xc0                           // END_COLLECTION
    };

    reportLength = reportDescriptor.size();
    return reportDescriptor.data();
}

const uint8_t* MultiHID::configuration_desc(uint8_t index)
{
    if (index != 0)
    {
        return nullptr;
    }

    const uint8_t DefaultConfiguration = 1;
    uint16_t TotalDescriptorLength = CONFIGURATION_DESCRIPTOR_LENGTH + INTERFACE_DESCRIPTOR_LENGTH
                                    + HID_DESCRIPTOR_LENGTH + 2 * ENDPOINT_DESCRIPTOR_LENGTH;
    constexpr uint8_t BmAttributes = (1U << 6U) | (1U << 7U);
    uint16_t HidVersion = HID_VERSION_1_11;
    uint16_t reportDescriptorLength = report_desc_length();
    uint16_t MaxHIDReportSize = MAX_HID_REPORT_SIZE;

    const std::vector<uint8_t> configurationDescriptorTemp
    {
        CONFIGURATION_DESCRIPTOR_LENGTH,    // bLength
        CONFIGURATION_DESCRIPTOR,           // bDescriptorType
        LOW(TotalDescriptorLength),         // wTotalLength (LSB)
        HIGH(TotalDescriptorLength),        // wTotalLength (MSB)
        0x01,                               // bNumInterfaces
        DefaultConfiguration,               // bConfigurationValue
        0x00,                               // iConfiguration
        BmAttributes,                       // bmAttributes
        C_POWER(0),                         // bMaxPower
        /************** Descriptor of joystick interface ****************/
        INTERFACE_DESCRIPTOR_LENGTH,        // bLength
        INTERFACE_DESCRIPTOR,               // bDescriptorType
        0x00,                               // bInterfaceNumber
        0x00,                               // bAlternateSetting
        0x01,                               // bNumEndpoints
        HID_CLASS,                          // bInterfaceClass
        HID_SUBCLASS_NONE,                  // bInterfaceSubClass
        HID_PROTOCOL_NONE,                  // bInterfaceProtocol
        0x00,                               // iInterface

        HID_DESCRIPTOR_LENGTH,              // bLength
        HID_DESCRIPTOR,                     // bDescriptorType
        LOW(HidVersion),                    // bcdHID (LSB)
        HIGH(HidVersion),                   // bcdHID (MSB)
        0x00,                               // bCountryCode
        0x01,                               // bNumDescriptors
        REPORT_DESCRIPTOR,                  // bDescriptorType
        LOW(reportDescriptorLength),        // wDescriptorLength (LSB)
        HIGH(reportDescriptorLength),       // wDescriptorLength (MSB)

        ENDPOINT_DESCRIPTOR_LENGTH,         // bLength
        ENDPOINT_DESCRIPTOR,                // bDescriptorType
        _int_in,                            // bEndpointAddress
        E_INTERRUPT,                        // bmAttributes
        LOW(MaxHIDReportSize),              // wMaxPacketSize (LSB)
        HIGH(MaxHIDReportSize),             // wMaxPacketSize (MSB)
        1,                                  // bInterval (milliseconds)

        ENDPOINT_DESCRIPTOR_LENGTH,         // bLength
        ENDPOINT_DESCRIPTOR,                // bDescriptorType
        _int_out,                           // bEndpointAddress
        E_INTERRUPT,                        // bmAttributes
        LOW(MaxHIDReportSize),              // wMaxPacketSize (LSB)
        HIGH(MaxHIDReportSize),             // wMaxPacketSize (MSB)
        1                                  // bInterval (milliseconds)
    };

    MBED_ASSERT(configurationDescriptorTemp.size() == sizeof(configurationDescriptor));
    memcpy(static_cast<void*>(configurationDescriptor), static_cast<const void*>(configurationDescriptorTemp.data()), sizeof(configurationDescriptor));
    return static_cast<const uint8_t*>(configurationDescriptor);
}

/*
* Get string product descriptor
*
* @returns pointer to the string product descriptor
*/
const uint8_t* MultiHID::string_iproduct_desc()
{
    static const std::vector<uint8_t> overriddenStringIproductDescriptor
    {
        0x1A,                                                       //bLength
        STRING_DESCRIPTOR,                                          //bDescriptorType 0x03
        'W', 0, 'r', 0, 'i', 0, 's', 0, 't', 0, 'B', 0, 'r', 0, 'e', 0, 'a', 0, 'k', 0, 'e', 0, 'r', 0 //bString iProduct - HID device
    };
    return overriddenStringIproductDescriptor.data();
}

/*
 * sends HID joystick report to PC
 */
bool MultiHID::sendReport(JoystickData& joystickData)
{
    HID_REPORT report;
    std::vector<uint8_t> reportData
    {
        LOW(joystickData.X),
        HIGH(joystickData.X),
        LOW(joystickData.Y),
        HIGH(joystickData.Y),
        LOW(joystickData.Z),
        HIGH(joystickData.Z),
        LOW(joystickData.Rz),
        HIGH(joystickData.Rz),
        LOW(joystickData.Rx),
        HIGH(joystickData.Rx),
        LOW(joystickData.Ry),
        HIGH(joystickData.Ry),
        LOW(joystickData.slider),
        HIGH(joystickData.slider),
        LOW(joystickData.dial),
        HIGH(joystickData.dial),
        joystickData.hat,
        *reinterpret_cast<uint8_t*>(&joystickData.buttons),
        *reinterpret_cast<uint8_t*>(&joystickData.buttons + 1),     // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        *reinterpret_cast<uint8_t*>(&joystickData.buttons + 2),     // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        *reinterpret_cast<uint8_t*>(&joystickData.buttons + 3)      // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    };

    memcpy(static_cast<void*>(report.data), reportData.data(), reportData.size());
    report.length = reportData.size();
    return send(&report);
}
