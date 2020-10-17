/*
 * USB.h
 *
 *  Created on: 17.10.2020
 *      Author: Marcin
 */

#ifndef USB_H_
#define USB_H_

#include <mbed.h>
#include "USBHID.h"

#define USB_VID     0x0483 //STElectronics
#define USB_PID     0x5710 //joystick in FS mode
#define USB_VER     0x0001 //Nucleo Yoke IMU ver. 1

struct JoystickData
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t Rx;
    int16_t Ry;
    int16_t Rz;
    int16_t slider;
    int16_t dial;
    uint8_t hat;
    uint32_t buttons;
};

class MultiHID : public USBHID
{
public:
    MultiHID(uint16_t vendorId, uint16_t productId, uint16_t productRelease, bool blocking = false);
    ~MultiHID() override;
    MultiHID(MultiHID const&) = delete;
    void operator=(MultiHID const&) = delete;
    MultiHID(MultiHID const&&) = delete;
    void operator=(MultiHID const&&) = delete;
    const uint8_t* report_desc() override; // returns pointer to the report descriptor; Warning: this method must store the length of the report descriptor in reportLength
    bool sendReport(JoystickData& joystickData);
protected:
    const uint8_t* configuration_desc(uint8_t index) override;   // Get configuration descriptor; returns pointer to the configuration descriptor
    const uint8_t* string_iproduct_desc() override;      // Get string product descriptor
private:
    static const size_t ConfigurationDescriptorSize{41};
    uint8_t configurationDescriptor[ConfigurationDescriptorSize]{0};   // NOLINT
};

#endif /* USB_H_ */