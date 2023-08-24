#pragma once

#include <etsi_its_denm_coding/OCTET_STRING.h>
#include <string>

namespace etsi_its_denm_conversion {

  void toRos_OCTET_STRING(const OCTET_STRING_t& _OCTET_STRING_in, std::string& OCTET_STRING_out) {
    for (int i = 0; i < _OCTET_STRING_in.size; i++)
      OCTET_STRING_out.push_back(_OCTET_STRING_in.buf[i]);
  }

  void toStruct_OCTET_STRING(const std::string& _OCTET_STRING_in, OCTET_STRING_t& OCTET_STRING_out) {
    const char *cstr = _OCTET_STRING_in.c_str();
    int ret_v = OCTET_STRING_fromString(&OCTET_STRING_out, cstr);
  }

}