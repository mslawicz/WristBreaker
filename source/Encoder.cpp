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

Mutex AS5048A::spiMutex;    //NOLINT(fuchsia-statically-constructed-objects,cppcoreguidelines-avoid-non-const-global-variables)

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
    static constexpr size_t BufferSize = 32; 

    I2C i2c(PB_9, PB_8);    //XXX check ports!
    constexpr uint8_t DeviceAddress = 0x6C;
    constexpr char AddressZMCO = 0x00;
    constexpr char AddressZPOS = 0x01;
    constexpr char AddressMANG = 0x05;
    constexpr char AddressCONF = 0x07;
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
    constexpr int DataLength = 8;       //8-bit transmission
    constexpr int Mode = 1;             //Mode 1: POL=0, PHA=1
    interface.format(DataLength, Mode);
}

void AS5048A::displayStatus()    //display status of the encoder chip
{ 
    spiMutex.lock();
    transmit(0x0001U, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x0000U, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x3FFDU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    transmit(0x3FFEU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    //now the read buffer contains the response of 0x3FFD command
    constexpr uint8_t CompHighBit = 3U;
    uint16_t compHigh = static_cast<uint16_t>(rdBuffer[0] >> CompHighBit) & 1U;
    constexpr uint8_t CompLowBit = 2U;
    uint16_t compLow = static_cast<uint16_t>(rdBuffer[0] >> CompLowBit) & 1U;
    constexpr uint8_t COFBit = 1U;
    uint16_t COF = static_cast<uint16_t>(rdBuffer[0] >> COFBit) & 1U;
    constexpr uint8_t OCFBit = 0U;
    uint16_t OCF = static_cast<uint16_t>(rdBuffer[0] >> OCFBit) & 1U;
    constexpr uint8_t Hundred = 100;
    constexpr uint8_t Word8Max = 0xFF;
    uint16_t AGCPct = Hundred * rdBuffer[1] / Word8Max;
    transmit(0x3FFFU, Access::Read);    //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    //now the read buffer contains the response of 0x3FFE command
    constexpr uint8_t EightBits = 8U;
    constexpr uint16_t Word14Max = 0x3FFFU;        //14-bit data mask
    uint16_t magnitude = static_cast<uint16_t>((rdBuffer[0] << EightBits) + rdBuffer[1]) & Word14Max;
    uint16_t magnitudePct = Hundred * magnitude / Word14Max;
    constexpr uint8_t ErrorFlagBit = 2U;
    uint16_t error = static_cast<uint16_t>(rdBuffer[0] >> ErrorFlagBit) & 1U;
    spiMutex.unlock();

    std::cout << "encoder AS5048A";
    std::cout << ", value=" << value;
    std::cout << ", error flag=" << error;
    std::cout << ", comp high=" << compHigh;
    std::cout << ", comp low=" << compLow;
    std::cout << ", COF=" << COF;
    std::cout << ", OCF=" << OCF;
    std::cout << ", AGC=" << std::dec << AGCPct << "%"; 
    std::cout << ", mag=" << std::dec << magnitudePct << "%" << std::endl;
}

//send 16-bit data to encoder / receive previously requested data
void AS5048A::transmit(uint16_t data, Access access)
{
    constexpr uint8_t AccessPosition = 14U;
    data |= static_cast<uint16_t>(static_cast<uint16_t>(access) << AccessPosition);
    constexpr uint8_t ParityPosition = 15U;
    data |= static_cast<uint16_t>(getParityBit<uint16_t>(data) << ParityPosition);
    wrBuffer[0] = HI8(data);
    wrBuffer[1] = LO8(data);
    //write data to encoder and read previously requested data
    interface.write(reinterpret_cast<char*>(wrBuffer), DataSize, reinterpret_cast<char*>(rdBuffer), DataSize);
}

//read angle from encoder chip and return last value 
float AS5048A::getValue()
{
    constexpr uint16_t command = 0x3FFF;        //command: read angle value
    spiMutex.lock();
    transmit(command, Access::Read);        //send command and read the response of the previous command
    constexpr uint8_t EightBits = 8U;
    uint16_t data = (rdBuffer[0] << EightBits) + rdBuffer[1];
    spiMutex.unlock();
    if(getParityBit<uint16_t>(data) == 0)
    {
        //parity bit OK
        constexpr uint16_t Mask14 = 0x3FFF;
        data &= Mask14;
        if(reverse)
        {
            data = Mask14 - data;
        }
        value = static_cast<float>(data) / static_cast<float>(Mask14 + 1U);     //range <0,1)
    }
    return value;
}    