#pragma once

namespace etsi_its_primitives_conversion {

  template <typename T>
  void toRos_BOOLEAN(const T& _BOOLEAN_in, uint8_t& BOOLEAN_out) {
    BOOLEAN_out = _BOOLEAN_in;
  }

  template <typename T>
  void toStruct_BOOLEAN(const uint8_t& _BOOLEAN_in, T& BOOLEAN_out) {
    BOOLEAN_out = _BOOLEAN_in;
  }

  template <typename T>
  void toRos_BOOLEAN(const T& _BOOLEAN_in, bool& BOOLEAN_out) {
    BOOLEAN_out = _BOOLEAN_in;
  }

  template <typename T>
  void toStruct_BOOLEAN(const bool& _BOOLEAN_in, T& BOOLEAN_out) {
    BOOLEAN_out = _BOOLEAN_in;
  }

}