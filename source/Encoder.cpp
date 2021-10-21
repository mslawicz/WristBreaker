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

AS5600::AS5600(PinName input, bool reverse) :
    analogInput(input),
    reverse(reverse)
{
}

float AS5600::getValue()
{
    float value = analogInput.read();
    if(reverse)
    {
        value = 1.0F - value;
    }
    return value;
}    

void AS5600::displayStatus()    //display status of the encoder chip
{ 
    std::cout << "encoder AS5600, value=" << getValue() << std::endl;
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

AS5048A::AS5048A(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, bool reverse) :
    interface(MOSI, MISO, SCLK, CS, use_gpio_ssel),
    reverse(reverse)
{
    const int DataLength = 8;       //8-bit transmission
    const int Mode = 1;             //Mode 1: POL=0, PHA=1
    interface.format(DataLength, Mode);
}

void AS5048A::displayStatus()    //display status of the encoder chip
{ 
    uint16_t compHigh{0};
    uint16_t compLow{0};
    uint16_t COF{0};
    uint16_t OCF{0};
    uint16_t AGC{0};
    uint16_t magnitude{0};
    uint16_t error{0};

    transmit(0x0001U, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x0000U, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x3FFDU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x3FFEU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    //now the read buffer contains the response of 0x3FFD command
    const uint8_t CompHighBit = 3U;
    compHigh = static_cast<uint16_t>(rdBuffer[0] >> CompHighBit) & 1U;
    const uint8_t CompLowBit = 2U;
    compLow = static_cast<uint16_t>(rdBuffer[0] >> CompLowBit) & 1U;
    const uint8_t COFBit = 1U;
    COF = static_cast<uint16_t>(rdBuffer[0] >> COFBit) & 1U;
    const uint8_t OCFBit = 0U;
    OCF = static_cast<uint16_t>(rdBuffer[0] >> OCFBit) & 1U;
    AGC = static_cast<uint16_t>(rdBuffer[1]);
    transmit(0x3FFFU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    //now the read buffer contains the response of 0x3FFE command
    const uint8_t Byte = 8U;
    const uint16_t DataMask = 0x3FFFU;        //14-bit data mask
    magnitude = static_cast<uint16_t>((rdBuffer[0] << Byte) + rdBuffer[1]) & DataMask;
    const uint8_t ErrorFlagBit = 2U;
    error = static_cast<uint16_t>(rdBuffer[0] >> ErrorFlagBit) & 1U;

    std::cout << "encoder AS5048A";
    std::cout << ", value=" << value;
    std::cout << ", error flag=" << error;
    std::cout << ", comp high=" << compHigh;
    std::cout << ", comp low=" << compLow;
    std::cout << ", COF=" << COF;
    std::cout << ", OCF=" << OCF;
    std::cout << ", AGC=" << std::hex << "0x" << AGC; 
    std::cout << ", mag=" << std::hex << "0x" << magnitude << std::endl;
}

//send 16-bit data to encoder / receive previously requested data
void AS5048A::transmit(uint16_t data, Access access)
{
    const uint8_t AccessPosition = 14U;
    data |= static_cast<uint16_t>(static_cast<uint16_t>(access) << AccessPosition);
    const uint8_t ParityPosition = 15U;
    data |= static_cast<uint16_t>(getParityBit<uint16_t>(data) << ParityPosition);
    wrBuffer[0] = HI8(data);
    wrBuffer[1] = LO8(data);
    //write data to encoder and read previously requested data
    interface.write(reinterpret_cast<char*>(wrBuffer), DataSize, reinterpret_cast<char*>(rdBuffer), DataSize);
}

//read angle from encoder chip and return last value 
float AS5048A::getValue()
{
    const uint16_t command = 0x3FFF;        //command: read angle value
    transmit(command, Access::Read);        //send command and read the response of the previous command
    const uint8_t Byte = 8U;
    uint16_t data = (rdBuffer[0] << Byte) + rdBuffer[1];
    if(getParityBit<uint16_t>(data) == 0)
    {
        //parity bit OK
        const uint16_t Mask14 = 0x3FFF;
        data &= Mask14;
        if(reverse)
        {
            data = Mask14 - data;
        }
        value = static_cast<float>(data) / static_cast<float>(Mask14 + 1U);     //range <0,1)
    }
    return value;
}    