/*
 * Encoder.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <mbed.h>
#include "Console.h"

class Encoder
{
public:
    Encoder() =default;
    virtual ~Encoder() =default;
    Encoder(Encoder const&) = default;
    Encoder& operator=(Encoder const&) = default;
    Encoder(Encoder&&) = default;
    Encoder& operator=(Encoder&&) = default;
    virtual float getValue() =0;
};

// 12-bit rotary magnetic encoder with analog output
class AS5600 : public Encoder
{
public:
    explicit AS5600(PinName input);
    float getValue() override { return analogInput.read(); }
    static void program(const CommandVector& /*cv*/);
private:
    AnalogIn analogInput;
};

// 14-bit rotary magnetic encoder with SPI output
class AS5048A : public Encoder
{
public:
    explicit AS5048A(PinName MOSI, PinName MISO, PinName SCLK, PinName CS);
    float getValue() override { return 0; }
    void test();    //XXX test
private:
    SPI interface;   
    static const int DataSize{2};
    uint8_t wrBuffer[DataSize];     //NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
    uint8_t rdBuffer[DataSize];     //NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
};

#endif /* ENCODER_H_ */