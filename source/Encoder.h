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

enum struct Access: uint8_t
{
    Write = 0,
    Read = 1
};

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
    void readRequest() {};
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
    float getValue() override { return value; }
    void readRequest();
    void test();    //XXX test
private:
    void transmit(uint16_t data, Access access, bool async = false);
    void onReceptionCallback(int event);
    SPI interface;   
    static const int DataSize{2};
    uint8_t wrBuffer[DataSize]{0};     //NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
    uint8_t rdBuffer[DataSize]{0};     //NOLINT(hicpp-avoid-c-arrays,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
    float value{0};        //normalized encoder value; range <0,1>
    DigitalOut testPin{PG_3};   //XXX test
    bool discardData{false};    //discard data flag; set for preventing corrupted value readouts
};

#endif /* ENCODER_H_ */