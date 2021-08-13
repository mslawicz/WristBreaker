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
#include "Filter.h"

class Encoder
{
public:
    explicit Encoder(size_t filterSize) : medianFilter(filterSize) {}
    virtual ~Encoder() =default;
    Encoder(Encoder const&) = default;
    Encoder& operator=(Encoder const&) = default;
    Encoder(Encoder&&) = default;
    Encoder& operator=(Encoder&&) = default;
    virtual float getValue() =0;
    virtual float getFilteredValue() = 0;
protected:
    MedianFilter medianFilter;      //NOLINT
};

// 12-bit rotary magnetic encoder with analog output
class AS5600 : public Encoder
{
public:
    explicit AS5600(PinName input, size_t filterSize);
    float getValue() override { return analogInput.read(); }
    float getFilteredValue() override { return medianFilter.getMedian(analogInput.read()); }
    static void program(const CommandVector& /*cv*/);
private:
    AnalogIn analogInput;
};

// 14-bit rotary magnetic encoder with SPI output
class AS5048A : public Encoder
{
public:
    explicit AS5048A(PinName MOSI, PinName MISO, PinName SCLK, PinName CS, size_t filterSize);
    float getValue() override { return 0; }
    float getFilteredValue() override { return 0; }
private:
    SPI interface;   
};

#endif /* ENCODER_H_ */