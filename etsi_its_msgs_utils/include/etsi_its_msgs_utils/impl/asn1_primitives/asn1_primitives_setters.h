/*
=============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
=============================================================================
*/

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_SETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_SETTERS_H

/**
 * @file impl/asn1_primitives_setters.h
 * @brief File containing setter-functions that are used in the context of ETIS ITS Messages related to ASN.1 datatype primitives
 */

/**
 * @brief Set a Bit String by a vector of bools
 *
 * @tparam T
 * @param bitstring BitString to set
 * @param bits vector of bools
 */
template <typename T>
inline void setBitString(T& bitstring, const std::vector<bool>& bits) {
  // bit string size
  const int bits_per_byte = 8;
  const int n_bytes = (bits.size() - 1) / bits_per_byte + 1;
  const int n_bits = n_bytes * bits_per_byte;

  // init output
  bitstring.bits_unused = n_bits - bits.size();
  bitstring.value = std::vector<uint8_t>(n_bytes);

  // loop over all bytes in reverse order
  for (int byte_idx = n_bytes - 1; byte_idx >= 0; byte_idx--) {

    // loop over bits in a byte
    for (int bit_idx_in_byte = bits_per_byte - 1; bit_idx_in_byte >= 0; bit_idx_in_byte--) {

      // map bit index in byte to bit index in total bitstring
      int bit_idx = (n_bytes - byte_idx - 1) * bits_per_byte + bit_idx_in_byte;
      if (byte_idx == 0 && bit_idx < bitstring.bits_unused) break;

      // set bit in output bitstring appropriately
      if (bit_idx < bits.size()) {
        bitstring.value[byte_idx] |= bits[bits_per_byte - bit_idx - 1] << bit_idx_in_byte;
      }
    }
  }
}

#endif // ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_SETTERS_H