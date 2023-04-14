#pragma once

#include <limits>
#include <stdexcept>

#include <etsi_its_cam_coding/INTEGER.h>

namespace etsi_its_cam_conversion {

  void convert_toRos(const INTEGER_t& _INTEGER_in, long& INTEGER_out) {
    int status = asn_INTEGER2long(&_INTEGER_in, &INTEGER_out);
    if (status != 0)
      throw std::range_error("Failed to convert INTEGER_t to long");
  }

  template <typename T>
  void convert_toRos(const long& _INTEGER_in, T& INTEGER_out) {
    if (std::numeric_limits<T>::max() < _INTEGER_in)
      throw std::range_error("Failed to convert long (" + std::to_string(_INTEGER_in) + ") to smaller integer type (max: " + std::to_string(std::numeric_limits<T>::max()) + ")");
    INTEGER_out = _INTEGER_in;
  }

  void convert_toC(const long& _INTEGER_in, INTEGER_t& INTEGER_out) {
    int status = asn_long2INTEGER(&INTEGER_out, _INTEGER_in);
    if (status != 0)
      throw std::range_error("Failed to convert long to INTEGER_t");
  }

  void convert_toC(const long& _INTEGER_in, long& INTEGER_out) {
    INTEGER_out = _INTEGER_in;
  }
  
  void convert_toC(const long& _INTEGER_in, unsigned long& INTEGER_out) {
    if (std::numeric_limits<unsigned long>::max() < _INTEGER_in) throw std::range_error("Failed to convert long to smaller unsigned long");
    INTEGER_out = _INTEGER_in;
  }
}