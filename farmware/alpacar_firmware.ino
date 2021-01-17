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

static const uint8_t hoge = 0;

#include "battery_indicator.hpp"
#include "motor_driver.hpp"
#include <Wire.h>

alpacar::BatteryIndicator bat_indicator(
    static_cast<uint8_t>(alpacar::AlpacarPin::BAT_IND_5),
    static_cast<uint8_t>(alpacar::AlpacarPin::BAT_IND_4),
    static_cast<uint8_t>(alpacar::AlpacarPin::BAT_IND_3),
    static_cast<uint8_t>(alpacar::AlpacarPin::BAT_IND_2),
    static_cast<uint8_t>(alpacar::AlpacarPin::BAT_IND_1)
);
alpacar::MotorDriver motor_driver(
    static_cast<uint8_t>(alpacar::AlpacarPin::MOT_EN),
    static_cast<uint8_t>(alpacar::AlpacarPin::MOT_DIR),
    static_cast<uint8_t>(alpacar::AlpacarPin::MOT_STP)
);

uint8_t chip_config = 0;

uint8_t error_code = 0;
char error_message[32];

void setup()
{
    delay(500);

    //          ICSP HEADER
    //          +---------+
    //          |         |       | chip_config | i2c_addr | 1 | 2 |
    //     MISO | |*|  *  | +5V   +-------------+----------+---+---+
    //          | |1|     |       |     0x00    |   0x50   | X | X |
    // (GND)SCK | |*| |*| | MOSI  |     0x01    |   0x51   | O | X |
    //          |     |2| |       |     0x02    |   0x52   | X | O |
    //      RST |  *  |*| | GND   |     0x03    |   0x53   | O | O |
    //          |         |
    //          +---------+
    // init pins to check chip config
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT_PULLUP);
    pinMode(MOSI, INPUT_PULLUP);
    digitalWrite(SCK, LOW);
    // check config
    chip_config |= digitalRead(MISO) == LOW ? 0x01 : 0x00;
    chip_config |= digitalRead(MOSI) == LOW ? 0x02 : 0x00;
    if (chip_config >= 0x03) { setError(0x01, "Invalid config."); }

    // init features
    bat_indicator.init();
    motor_driver.init();
    motor_driver.enable(true);

    // start LED sequence
    for (uint8_t i=0x01; i<0x20; i=(i<<1)|1) {
        bat_indicator.set_led(i);
        delay(100);
    }
    bat_indicator.set_led(0);
    delay(300);
    bat_indicator.set_led(0x1f);
    delay(200);
    bat_indicator.set_led(0);
    delay(200);
    bat_indicator.set_led(0x1f);
    delay(200);
    bat_indicator.set_led(0);
    // delay(200);
    // bat_indicator.set_led(0x01 << chip_config);
    // delay(500);
    // bat_indicator.set_led(0);

    // init I2C
    uint8_t tmp_addr = (uint8_t)(0x50 + chip_config);
    Wire.begin(tmp_addr);
    TWAR = (tmp_addr << 1) | 1; // enable broadcasts to be received
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
}

void loop()
{
    motor_driver.lowSpeedLoop(false);
    if (error_code != 0) {
        motor_driver.enable(false);
        while (true) {
            bat_indicator.set_led(error_code);
            delay(500);
            bat_indicator.set_led(0);
            delay(500);
        }
    }
}

void onReceive(int nbytes)
{
    if (nbytes != 10) {
        char tmp_message[] = "Received       bytes";
        copyUIntTo(tmp_message + 9, nbytes);
        setError(0x02, tmp_message);
        return;
    }

    // empty read
    for (uint8_t i=0; i<chip_config*3; ++i) { Wire.read(); }
    // read wheel target speed
    uint8_t dir = Wire.read();
    uint16_t spd = (Wire.read() << 8) + Wire.read();
    // set wheel speed
    if (dir == 0 || dir == 1) {
        motor_driver.setSpeed(dir == 0 ? true : false, spd);
    } else {
        motor_driver.enable(false);
    }
    // empty read
    for (uint8_t i=0; i<(2-chip_config)*3; ++i) { Wire.read(); }

    // read SOC
    uint8_t soc = Wire.read();
    if (soc <= 100) {
        // set LED
        bat_indicator.set_soc(soc);
    }
}

void onRequest()
{
    Wire.write(error_code);
    for (uint8_t i=0; i<32; ++i) { Wire.write(error_message[i]); }
}

void setError(const uint8_t code, const char* message) {
    for (uint8_t i=0; i<32; ++i) { error_message[i] = '\0'; }

    if (code == 0 || code >= 0x1f) {
        // invalid message
        char tmp_msg[] = "Invalid error: 0x00";
        tmp_msg[17] = to1Hex(code >> 4);
        tmp_msg[18] = to1Hex(code);
        error_code = 0x1f;
        for (uint8_t i=0; tmp_msg[i]!='\0'; ++i) {
            error_message[i] = tmp_msg[i];
        }
    } else {
        error_code = code;
        for (uint8_t i=0; i < 32 && message[i]!='\0'; ++i) {
            error_message[i] = message[i];
        }
    }
}

char to1Hex(const uint8_t val) {
    if ((val & 0x0f) >= 0x0a) { return 'a' + (val & 0x0f) - 0x0a; }
    else                      { return '0' + (val & 0x0f); }
}

void copyUIntTo(char* ptr, uint16_t num)
{
    for (uint8_t i=0; i<5; ++i) {
        ptr[i] = to1Hex(num % 10);
        if (num == 0) { break; }
        num /= 10;
    }
}
