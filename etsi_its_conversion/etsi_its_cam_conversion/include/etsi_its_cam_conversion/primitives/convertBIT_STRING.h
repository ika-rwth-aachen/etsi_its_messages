#pragma once

#include <algorithm>
#include <boost/array.hpp>
#include <etsi_its_cam_coding/BIT_STRING.h>


namespace etsi_its_cam_conversion {

  void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, std::vector<uint8_t>& BIT_STRING_out) {
    
    // bit string size
    const int bits_per_byte = 8;
    const int n_bytes = _BIT_STRING_in.size;
    const int n_bits = n_bytes * bits_per_byte;

    // init output
    BIT_STRING_out.resize(n_bits - _BIT_STRING_in.bits_unused, 0);

    // loop over bytes in reverse order
    for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {

      // loop over bits in a byte
      for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {

        // map bit index in byte to bit index in total bitstring
        int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
        if (byte_idx == 0 && bit_idx >= n_bits - _BIT_STRING_in.bits_unused) break;

        // extract bit from bitstring and set output array entry appropriately
        bool byte_has_true_bit = _BIT_STRING_in.buf[byte_idx] & (1 << bit_idx_in_byte);
        if (byte_has_true_bit) BIT_STRING_out[bit_idx] = 1;
      }
    }
  }

  void convert_BIT_STRINGtoC(const std::vector<uint8_t>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out) {
    
    // bit string size
    const int bits_per_byte = 8;
    const int n_bytes = (_BIT_STRING_in.size() - 1) / bits_per_byte + 1;
    const int n_bits = n_bytes * bits_per_byte;

    // init output
    BIT_STRING_out.bits_unused = n_bits - _BIT_STRING_in.size();
    BIT_STRING_out.size = n_bytes;
    BIT_STRING_out.buf = new uint8_t[BIT_STRING_out.size];
    std::memset(BIT_STRING_out.buf, 0, BIT_STRING_out.size);

    // loop over all bytes in reverse order
    for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {

      // loop over bits in a byte
      for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {

        // map bit index in byte to bit index in total bitstring
        int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
        if (byte_idx == 0 && bit_idx >= n_bits - BIT_STRING_out.bits_unused) break;

        // set bit in output bitstring appropriately
        BIT_STRING_out.buf[byte_idx] |= _BIT_STRING_in[bit_idx] << bit_idx_in_byte;
      }
    }
  }

  template <std::size_t N>
  void convert_BIT_STRINGtoRos(const BIT_STRING_t& _BIT_STRING_in, boost::array<uint8_t, N>& BIT_STRING_out) {

    std::vector<uint8_t> BIT_STRING_out_vector;
    convert_BIT_STRINGtoRos(_BIT_STRING_in, BIT_STRING_out_vector);
    std::copy(BIT_STRING_out_vector.begin(), BIT_STRING_out_vector.end(), BIT_STRING_out.begin());
  }


  template <std::size_t N>
  void convert_BIT_STRINGtoC(const boost::array<uint8_t, N>& _BIT_STRING_in, BIT_STRING_t& BIT_STRING_out) {

    std::vector<uint8_t> _BIT_STRING_in_vector(N);
    std::copy(_BIT_STRING_in.begin(), _BIT_STRING_in.end(), _BIT_STRING_in_vector.begin());
    convert_BIT_STRINGtoC(_BIT_STRING_in_vector, BIT_STRING_out);
  }

}