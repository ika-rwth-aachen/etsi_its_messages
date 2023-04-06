#pragma once

#include <boost/array.hpp>
#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_cam_coding/OCTET_STRING.h>


namespace etsi_its_cam_conversion {

  template <std::size_t N>
  void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, boost::array<uint8_t, N>& BIT_STRING_out) {
    BIT_STRING_out.assign(0);
    const int bits_per_byte = 8;
    const int n_bytes = _BIT_STRING_in.size;
    const int n_bits = n_bytes * bits_per_byte;
    for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {
      for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {
        int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
        if (byte_idx == 0 && bit_idx >= n_bits - _BIT_STRING_in.bits_unused) break;
        bool byte_has_true_bit = _BIT_STRING_in.buf[byte_idx] & (1 << bit_idx_in_byte);
        if (byte_has_true_bit) BIT_STRING_out[bit_idx] = 1;
      }
    }
  }


  template <std::size_t N>
  void convert_BIT_STRINGtoC(const boost::array<uint8_t, N>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out) {
    const int bits_per_byte = 8;
    const int n_bytes = (N - 1) / bits_per_byte + 1;
    const int n_bits = n_bytes * bits_per_byte;
    BIT_STRING_out.bits_unused = n_bits - N;
    BIT_STRING_out.size = n_bytes;
    BIT_STRING_out.buf = new uint8_t[BIT_STRING_out.size];
    std::memset(BIT_STRING_out.buf, 0, BIT_STRING_out.size);
    for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {
      for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {
        int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
        if (byte_idx == 0 && bit_idx >= n_bits - BIT_STRING_out.bits_unused) break;
        BIT_STRING_out.buf[byte_idx] |= _BIT_STRING_in[bit_idx] << bit_idx_in_byte;
      }
    }
  }

  void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, std::vector<uint8_t>& BIT_STRING_out) {
    // TODO
  }

  void convert_BIT_STRINGtoC(const std::vector<uint8_t>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out) {
    // TODO
  }

}