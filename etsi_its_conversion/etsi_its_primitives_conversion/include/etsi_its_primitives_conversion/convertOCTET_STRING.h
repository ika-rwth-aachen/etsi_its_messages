#pragma once

#include <cstring>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_OCTET_STRING(const T& _OCTET_STRING_in, std::vector<uint8_t>& OCTET_STRING_out) {

    OCTET_STRING_out.resize(_OCTET_STRING_in.size);
    std::memcpy(OCTET_STRING_out.data(), _OCTET_STRING_in.buf, _OCTET_STRING_in.size);
  }

  template <typename T>
  void toStruct_OCTET_STRING(const std::vector<uint8_t>& _OCTET_STRING_in, T& OCTET_STRING_out) {

    OCTET_STRING_out.size = _OCTET_STRING_in.size();
    OCTET_STRING_out.buf = new uint8_t[OCTET_STRING_out.size];
    std::memcpy(OCTET_STRING_out.buf, _OCTET_STRING_in.data(), OCTET_STRING_out.size);
  }

  template <typename T>
  void toStruct_OCTET_STRING(const std::string& _OCTET_STRING_in, T& OCTET_STRING_out) {

    const char *cstr = _OCTET_STRING_in.c_str();
    int status = OCTET_STRING_fromString(&OCTET_STRING_out, cstr);
    if (status != 0) throw std::invalid_argument("Failed to convert string to OCTET STRING");
  }

}