/** ============================================================================
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
============================================================================= */

#pragma once

#include <limits>
#include <stdexcept>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_INTEGER(const T& _INTEGER_in, int64_t& INTEGER_out) {
    long out;
    int status = asn_INTEGER2long(&_INTEGER_in, &out);
    if (status != 0)
      throw std::range_error("Failed to convert INTEGER_t to int64_t");
    INTEGER_out = static_cast<int64_t>(out);
  }

  template <typename T>
  void toRos_INTEGER(const T& _INTEGER_in, uint64_t& INTEGER_out) {
    unsigned long out;
    int status = asn_INTEGER2ulong(&_INTEGER_in, &out);
    if (status != 0)
      throw std::range_error("Failed to convert INTEGER_t to uint64_t");
    INTEGER_out = static_cast<uint64_t>(out);
  }

  template <typename T>
  void toRos_INTEGER(const long& _INTEGER_in, T& INTEGER_out) {
    if (std::numeric_limits<T>::max() < _INTEGER_in)
      throw std::range_error("Failed to convert long (" + std::to_string(_INTEGER_in) + ") to smaller integer type (max: " + std::to_string(std::numeric_limits<T>::max()) + ")");
    INTEGER_out = static_cast<T>(_INTEGER_in);
  }

  template <typename T>
  void toStruct_INTEGER(const int64_t& _INTEGER_in, T& INTEGER_out) {
    const long in = static_cast<long>(_INTEGER_in);
    int status = asn_long2INTEGER(&INTEGER_out, in);
    if (status != 0)
      throw std::range_error("Failed to convert int64_t to INTEGER_t");
  }

  void toStruct_INTEGER(const int64_t& _INTEGER_in, long& INTEGER_out) {
    INTEGER_out = _INTEGER_in;
  }

  void toStruct_INTEGER(const int64_t& _INTEGER_in, unsigned long& INTEGER_out) {
    if (_INTEGER_in < 0)
      throw std::range_error("Failed to convert int64_t to unsigned long");
    INTEGER_out = static_cast<unsigned long>(_INTEGER_in);
  }
}