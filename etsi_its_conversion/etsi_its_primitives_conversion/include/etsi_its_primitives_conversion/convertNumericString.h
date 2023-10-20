#pragma once

#include <string>
#include <etsi_its_primitives_conversion/convertOCTET_STRING.h>

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_NumericString(const T& _NumericString_in, std::string& NumericString_out) {
    toRos_OCTET_STRING(_NumericString_in, NumericString_out);
  }

  template <typename T>
  void toStruct_NumericString(const std::string& _NumericString_in, T& NumericString_out) {
    toStruct_OCTET_STRING(_NumericString_in, NumericString_out);
  }

}