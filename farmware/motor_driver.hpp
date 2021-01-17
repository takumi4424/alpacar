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

#ifndef MOTOR_DRIVER_HPP_89E5D79B_30AC_44C2_BC5E_2A1BEEBB1228
#define MOTOR_DRIVER_HPP_89E5D79B_30AC_44C2_BC5E_2A1BEEBB1228

// See https://github.com/makerbase-mks/MKS-SERVO42A

#include "pins.hpp"

namespace alpacar {

class MotorDriver
{
public:
    MotorDriver(const uint8_t en_pin, const uint8_t dir_pin, const uint8_t spd_pin)
            : en_pin_(en_pin)
            , dir_pin_(dir_pin)
            , spd_pin_(spd_pin)
            , initialized_(false)
    {}

    void init(const bool enbl=true)
    {
        pinMode(static_cast<uint8_t>(AlpacarPin::MOT_DIR), OUTPUT);
        pinMode(static_cast<uint8_t>(AlpacarPin::MOT_STP), OUTPUT);
        pinMode(static_cast<uint8_t>(AlpacarPin::MOT_EN),  OUTPUT);
        if (!initialized_) {
            this->simpleEnable(false);
            this->simpleSetDirection(true);
            this->simpleSetSpeed(0);
            this->initialized_ = true;
        }
        this->enable(enbl);
    }

    void enable(const bool enbl)
    {
        this->simpleEnable(enbl);
        this->simpleSetDirection(true);
        this->simpleSetSpeed(0);
    }

    inline void setSpeed(const bool direction, const uint16_t speed)
    {
        if (!this->enabled_)               { this->simpleEnable(true); }
        if (this->direction_ != direction) { this->simpleSetDirection(direction); }
        if (this->speed_     != speed)     { this->simpleSetSpeed(speed); }
    }

    void lowSpeedLoop(const bool loop_internal=true)
    {
        const uint16_t delay_interval[] = {
            1000 - 1 /*  1Hz */, 500 - 1 /*  2Hz */, 333 - 1 /*  3Hz */, 250 - 1 /*  4Hz */, 200 - 1 /*  5Hz */, 166 - 1 /*  6Hz */,
            142  - 1 /*  7Hz */, 125 - 1 /*  8Hz */, 111 - 1 /*  9Hz */, 100 - 1 /* 10Hz */, 90  - 1 /* 11Hz */, 83  - 1 /* 12Hz */,
            76   - 1 /* 13Hz */, 71  - 1 /* 14Hz */, 66  - 1 /* 15Hz */, 62  - 1 /* 16Hz */, 58  - 1 /* 17Hz */, 55  - 1 /* 18Hz */,
            52   - 1 /* 19Hz */, 50  - 1 /* 20Hz */, 47  - 1 /* 21Hz */, 45  - 1 /* 22Hz */, 43  - 1 /* 23Hz */, 41  - 1 /* 24Hz */,
            40   - 1 /* 25Hz */, 38  - 1 /* 26Hz */, 37  - 1 /* 27Hz */, 35  - 1 /* 28Hz */, 34  - 1 /* 29Hz */, 33  - 1 /* 30Hz */,
        };

        bool tmp = true;
        while (tmp) {
            tmp = loop_internal;

            uint16_t tmp_spd = this->speed_;
            if (tmp_spd == 0 || tmp_spd >= 31) { continue; }
            digitalWrite(this->spd_pin_, HIGH);
            delay(1);
            digitalWrite(this->spd_pin_, LOW);
            delay(delay_interval[tmp_spd - 1]);
        }
    }
private:
    const uint8_t en_pin_;
    const uint8_t dir_pin_;
    const uint8_t spd_pin_;
    bool initialized_;
    bool enabled_;
    bool direction_;
    uint16_t speed_;

    inline void simpleEnable(const bool enbl)
    {
        digitalWrite(this->en_pin_, enbl ? LOW : HIGH);
        this->enabled_ = enbl;
    }

    inline void simpleSetDirection(const bool direction)
    {
        digitalWrite(this->dir_pin_, direction ? HIGH : LOW);
        this->direction_ = direction;
    }

    inline void simpleSetSpeed(const uint16_t speed)
    {
        if (speed < 31) { noTone(this->spd_pin_); }
        else            { tone(  this->spd_pin_, speed); }
        this->speed_ = speed;
    }
};

} // namespace alpacar

#endif // MOTOR_DRIVER_HPP_89E5D79B_30AC_44C2_BC5E_2A1BEEBB1228
