/*
 * Encoder.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Encoder.h"
#include "Convert.h"
#include <iostream>
#include <iomanip>
#include <chrono>

AS5600::AS5600(PinName input) :
    analogInput(input)
{
}

// program the encoder AS5600 chip via I2C
void AS5600::program(const CommandVector& /*cv*/)
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

AS5048A::AS5048A(PinName MOSI, PinName MISO, PinName SCLK, PinName CS) :
    interface(MOSI, MISO, SCLK, CS, use_gpio_ssel)
{
    const int DataLength = 8;       //8-bit transmission
    const int Mode = 1;             //Mode 1: POL=0, PHA=1
    interface.format(DataLength, Mode);
}

void AS5048A::test()    //XXX test
{ 
    transmit(0x3FFD, Access::Read);
    transmit(0x3FFE, Access::Read);
    transmit(0x3FFF, Access::Read);
}

//send 16-bit data to encoder / receive previously requested data
void AS5048A::transmit(uint16_t data, Access access, bool async)
{
    const uint8_t AccessPosition = 14U;
    data |= static_cast<uint16_t>(static_cast<uint16_t>(access) << AccessPosition);
    const uint8_t ParityPosition = 15U;
    data |= static_cast<uint16_t>(getParityBit<uint16_t>(data) << ParityPosition);
    wrBuffer[0] = HI8(data);
    wrBuffer[1] = LO8(data);
    if(async)
    {
        //initialize asynchronously data transfer / read data in callback function
        interface.transfer<uint8_t>(reinterpret_cast<uint8_t*>(wrBuffer), DataSize, reinterpret_cast<uint8_t*>(rdBuffer), DataSize, callback(this, &AS5048A::onReceptionCallback));
    }
    else
    {
        //write data to encoder and read previously requested data
        interface.write(reinterpret_cast<char*>(wrBuffer), DataSize, reinterpret_cast<char*>(rdBuffer), DataSize);
    }
}

//callback on asynchronous data reception
void AS5048A::onReceptionCallback(int event)
{
    if(event == SPI_EVENT_COMPLETE)     //NOLINT(hicpp-signed-bitwise)
    {
        const uint8_t Byte = 8U;
        uint16_t data = (rdBuffer[0] << Byte) + rdBuffer[1];
        if(getParityBit<uint16_t>(data) == 0)
        {
            //parity bit OK and data should not be discarded
            const uint16_t Mask14 = 0x3FFF;
            value = static_cast<float>(data & Mask14) / static_cast<float>(Mask14);
        }
    }
}

//request of asynchronous encoder readback
void AS5048A::readRequest()
{
    const uint16_t command = 0x3FFF;        //command: read angle value
    transmit(command, Access::Read, true);  //asynchronous read
}