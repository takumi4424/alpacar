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

#ifndef PINS_HPP_919A7BA8_D757_4192_859F_E18F580FE01D
#define PINS_HPP_919A7BA8_D757_4192_859F_E18F580FE01D

namespace alpacar {

enum class AlpacarPin : uint8_t
{
    BAT_IND_1 = 0,
    BAT_IND_2 = 1,
    BAT_IND_3 = 2,
    BAT_IND_4 = 3,
    BAT_IND_5 = 4,

    MOT_DIR = A1,
    MOT_STP = A2,
    MOT_EN  = A3,
};

} // namespace alpacar


#endif // PINS_HPP_919A7BA8_D757_4192_859F_E18F580FE01D
