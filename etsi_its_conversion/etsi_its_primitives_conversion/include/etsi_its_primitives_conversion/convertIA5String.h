#pragma once

#include <string>
#include <etsi_its_primitives_conversion/convertOCTET_STRING.h>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_IA5String(const T& _IA5String_in, std::string& IA5String_out) {
    toRos_OCTET_STRING(_IA5String_in, IA5String_out);
  }

  template <typename T>
  void toStruct_IA5String(const std::string& _IA5String_in, T& IA5String_out) {
    toStruct_OCTET_STRING(_IA5String_in, IA5String_out);
  }

}