#pragma once

#include <string>
#include <etsi_its_primitives_conversion/convertOCTET_STRING.h>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_UTF8String(const T& _UTF8String_in, std::string& UTF8String_out) {
    std::stringstream ss;
    for (int i = 0; i < _UTF8String_in.size; i++) {
      ss << _UTF8String_in.buf[i];
    }
    UTF8String_out = ss.str();
  }

  template <typename T>
  void toStruct_UTF8String(const std::string& _UTF8String_in, T& UTF8String_out) {
    toStruct_UTF8String(_UTF8String_in, UTF8String_out);
  }

}