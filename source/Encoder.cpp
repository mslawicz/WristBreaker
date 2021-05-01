/*
 * Encoder.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Encoder.h"
#include <iostream>
#include <iomanip>
#include <chrono>

AS5600::AS5600(PinName input) :
    analogInput(input)
{
}

// program the encoder chip via I2C
void Encoder::program(const CommandVector& /*cv*/)
{
    static const size_t BufferSize = 32; 

    I2C i2c(PB_9, PB_8);    //XXX check ports!
    const uint8_t DeviceAddress = 0x6C;
    const char AddressZMCO = 0x00;
    const char AddressZPOS = 0x01;
    const char AddressMANG = 0x05;
    const char AddressCONF = 0x07;
    char dataBuffer[BufferSize];       //NOLINT
    if(i2c.write(DeviceAddress, &AddressZMCO, 1, true) == 0)
    {
        //device has responded - ready to read
        if(i2c.read(DeviceAddress, static_cast<char*>(dataBuffer), BufferSize) == 0)
        {
            std::cout << "Registers: ";
            for(char byte : dataBuffer)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " "; //NOLINT
            }
            std::cout << std::endl;

        //     dataBuffer[0] = AddressZPOS;
        //     dataBuffer[1] = 0x00;
        //     dataBuffer[2] = 0x00;
        //     dataBuffer[3] = 0x0F;
        //     dataBuffer[4] = 0xFF;
        //     i2c.write(DeviceAddress, static_cast<char*>(dataBuffer), 5);
        //     ThisThread::sleep_for(std::chrono::milliseconds{1});

        //     i2c.write(DeviceAddress, &AddressZMCO, 1, true);
        //     i2c.read(DeviceAddress, static_cast<char*>(dataBuffer), BufferSize);
        //     std::cout << "Registers: ";
        //     for(char byte : dataBuffer)
        //     {
        //         std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " "; //NOLINT
        //     }
        //     std::cout << std::endl;
        // }
        // else
        // {
        //     //no acknowledge from device
        //     std::cout << "failed to read from i2c device with address 0x" << std::hex << static_cast<int>(DeviceAddress) << std::endl;
        }
    }
    else
    {
        //no acknowledge from device
        std::cout << "i2c device with address 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(DeviceAddress) << " not found\n";
    }

}