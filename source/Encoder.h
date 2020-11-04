/*
 * Encoder.h
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <mbed.h>

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

class AS5600 : public Encoder
{
public:
    explicit AS5600(PinName input);
    float getValue() override { return analogInput.read(); }
private:
    AnalogIn analogInput;
};

class AS5480 : public Encoder
{
public:
    float getValue() override { return 0; }
};

#endif /* ENCODER_H_ */