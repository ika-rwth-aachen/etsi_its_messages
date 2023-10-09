#pragma once

#include <cstring>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_BIT_STRING(const T& _BIT_STRING_in, std::vector<uint8_t>& BIT_STRING_out) {

    BIT_STRING_out.resize(_BIT_STRING_in.size);
    std::memcpy(BIT_STRING_out.data(), _BIT_STRING_in.buf, _BIT_STRING_in.size);
  }

  template <typename T>
  void toStruct_BIT_STRING(const std::vector<uint8_t>& _BIT_STRING_in, T& BIT_STRING_out) {

    BIT_STRING_out.size = _BIT_STRING_in.size();
    BIT_STRING_out.buf = new uint8_t[BIT_STRING_out.size];
    std::memcpy(BIT_STRING_out.buf, _BIT_STRING_in.data(), BIT_STRING_out.size);
  }

}