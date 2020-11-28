/*
 * Encoder.cpp
 *
 *  Created on: 03.11.2020
 *      Author: Marcin
 */

#include "Encoder.h"
#include <iostream>

AS5600::AS5600(PinName input) :
    analogInput(input)
{
}

// program the encoder chip via I2C
void Encoder::program(const CommandVector& /*cv*/)
{
    std::cout << "programming encoder\n";
}