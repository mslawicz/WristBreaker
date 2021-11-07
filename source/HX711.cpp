#include "HX711.h"

/*
create HX711 object and set interrupt
use unique dataPin bus!
*/
HX711::HX711(PinName dataPin, PinName clockPin, uint8_t totalPulses) :
    dataPin(dataPin, PinMode::PullUp),
    clock(clockPin, 0),
    totalPulses(totalPulses)
{
    MBED_ASSERT((totalPulses >= 25) && (totalPulses <= 27));
    this->dataPin.fall(callback(this, &HX711::intHandler));
}

/*
read data from the HX711 chip
*/
void HX711::intHandler()
{
    constexpr uint8_t DataBits = 24;
    uint32_t dataBuffer = 0;
    dataPin.fall(nullptr);
    // generate clock pulses and read data bits
    for(uint8_t pulse = 0; pulse < totalPulses; pulse++)
    {
        // the pulse mustn't be interrupted (maximum length 50 us)
        CriticalSectionLock lock;
        clock = 1;
        if(pulse < DataBits)      // read 24 bits of data
        {
            dataBuffer = (dataBuffer << 1) | dataPin.read();    //NOLINT(hicpp-signed-bitwise)
        }
        clock = 0;
    }

    dataRegister = dataBuffer;
    dataPin.fall(callback(this, &HX711::intHandler));
}

