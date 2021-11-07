#ifndef HX711_H_
#define HX711_H_

#include <mbed.h>

constexpr uint8_t HX711Gain128 = 25;
constexpr uint8_t HX711Gain32 = 26;
constexpr uint8_t HX711Gain64 = 27;

class HX711
{
public:
    HX711(PinName dataPin, PinName clockPin, uint8_t totalPulses = HX711Gain128);
    uint32_t getDataRegister() const { return dataRegister; }
    float getValue() const { return static_cast<float>(static_cast<int32_t>(dataRegister << 8U)) / static_cast<float>(0x7FFFFF00); } //NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
private:
    void intHandler();          // reads data from HX711 chip
    InterruptIn dataPin;        // data pin
    DigitalOut clock;           // clock pin
    uint8_t totalPulses;        // number of pulses to generate <25-27>
    uint32_t dataRegister{0};   // data read from the chip
};

#endif /* HX711_H_ */