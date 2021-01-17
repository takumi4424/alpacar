// MIT License
//
// Copyright (c) 2021 Takumi Kodama
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BATTERY_INDICATOR_HPP_94BD6458_357F_437F_A12A_22FCC2294FDC
#define BATTERY_INDICATOR_HPP_94BD6458_357F_437F_A12A_22FCC2294FDC

#include "pins.hpp"

namespace alpacar {

class BatteryIndicator
{
public:
    BatteryIndicator(const uint8_t pin0, const uint8_t pin1, const uint8_t pin2, const uint8_t pin3, const uint8_t pin4)
            : pin0_(pin0)
            , pin1_(pin1)
            , pin2_(pin2)
            , pin3_(pin3)
            , pin4_(pin4)
    {}

    void init()
    {
        pinMode(this->pin0_, OUTPUT);
        pinMode(this->pin1_, OUTPUT);
        pinMode(this->pin2_, OUTPUT);
        pinMode(this->pin3_, OUTPUT);
        pinMode(this->pin4_, OUTPUT);
    }

    inline void set_soc(const uint8_t soc)
    {
        digitalWrite(this->pin0_, (soc > 0            ) ? HIGH : LOW);
        digitalWrite(this->pin1_, (soc > 0 && soc > 20) ? HIGH : LOW);
        digitalWrite(this->pin2_, (soc > 0 && soc > 40) ? HIGH : LOW);
        digitalWrite(this->pin3_, (soc > 0 && soc > 60) ? HIGH : LOW);
        digitalWrite(this->pin4_, (soc > 0 && soc > 80) ? HIGH : LOW);
    }

    inline void set_led(const uint8_t led)
    {
        digitalWrite(this->pin0_, (led & 0x01) != 0 ? HIGH : LOW);
        digitalWrite(this->pin1_, (led & 0x02) != 0 ? HIGH : LOW);
        digitalWrite(this->pin2_, (led & 0x04) != 0 ? HIGH : LOW);
        digitalWrite(this->pin3_, (led & 0x08) != 0 ? HIGH : LOW);
        digitalWrite(this->pin4_, (led & 0x10) != 0 ? HIGH : LOW);
    }
private:
    const uint8_t pin0_;
    const uint8_t pin1_;
    const uint8_t pin2_;
    const uint8_t pin3_;
    const uint8_t pin4_;
};

} // namespace alpacar

#endif // BATTERY_INDICATOR_HPP_94BD6458_357F_437F_A12A_22FCC2294FDC
