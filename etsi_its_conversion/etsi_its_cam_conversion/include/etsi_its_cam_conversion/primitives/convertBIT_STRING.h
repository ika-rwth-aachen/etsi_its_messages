#pragma once

#include <cstring>

#include <etsi_its_cam_coding/BIT_STRING.h>


namespace etsi_its_cam_conversion {

  void toRos_BIT_STRING(const BIT_STRING_t& _BIT_STRING_in, std::vector<uint8_t>& BIT_STRING_out) {

    BIT_STRING_out.resize(_BIT_STRING_in.size);
    std::memcpy(BIT_STRING_out.data(), _BIT_STRING_in.buf, _BIT_STRING_in.size);
  }

  void toStruct_BIT_STRING(const std::vector<uint8_t>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out) {

    BIT_STRING_out.size = _BIT_STRING_in.size();
    BIT_STRING_out.buf = new uint8_t[BIT_STRING_out.size];
    std::memcpy(BIT_STRING_out.buf, _BIT_STRING_in.data(), BIT_STRING_out.size);
  }

}