/*
=============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================
*/

/**
 * @file impl/J2735/j2735_v2x_dict_getters.h
 * @brief Getter functions for the J2735 V2X Communications Message Set Dictionary
 */

#pragma once

namespace etsi_its_msgs {

namespace J2735_access {

  inline bool setMinuteOfTheYear(MinuteOfTheYear& moy, const uint32_t moy_value) {
    throwIfOutOfRange(moy_value, MinuteOfTheYear::MIN, MinuteOfTheYear::MAX, "MinuteOfTheYear");
    moy.value = moy_value;
    return true;
  }

} // namespace J2735_access

} // namespace etsi_its_msgs
