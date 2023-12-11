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

#include <cstring>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_OCTET_STRING(const T& _OCTET_STRING_in, std::vector<uint8_t>& OCTET_STRING_out) {

    OCTET_STRING_out.resize(_OCTET_STRING_in.size);
    std::memcpy(OCTET_STRING_out.data(), _OCTET_STRING_in.buf, _OCTET_STRING_in.size);
  }

  template <typename T>
  void toRos_OCTET_STRING(const T& _OCTET_STRING_in, std::string& OCTET_STRING_out) {

    std::stringstream ss;
    for (int i = 0; i < _OCTET_STRING_in.size; i++) {
      ss << _OCTET_STRING_in.buf[i];
    }
    OCTET_STRING_out = ss.str();
  }

  template <typename T>
  void toStruct_OCTET_STRING(const std::vector<uint8_t>& _OCTET_STRING_in, T& OCTET_STRING_out) {

    OCTET_STRING_out.size = _OCTET_STRING_in.size();
    OCTET_STRING_out.buf = new uint8_t[OCTET_STRING_out.size];
    std::memcpy(OCTET_STRING_out.buf, _OCTET_STRING_in.data(), OCTET_STRING_out.size);
  }

  template <typename T>
  void toStruct_OCTET_STRING(const std::string& _OCTET_STRING_in, T& OCTET_STRING_out) {

    OCTET_STRING_t* octet_string = OCTET_STRING_new_fromBuf(&asn_DEF_OCTET_STRING, _OCTET_STRING_in.c_str(), _OCTET_STRING_in.size());
    OCTET_STRING_out = *octet_string;
  }

}