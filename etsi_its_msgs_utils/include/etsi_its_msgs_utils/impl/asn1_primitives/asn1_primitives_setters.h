// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

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
  const int n_bytes = static_cast<int>((bits.size() - 1) / bits_per_byte) + 1;
  const int n_bits = n_bytes * bits_per_byte;

  // init output
  bitstring.bits_unused = n_bits - bits.size();
  bitstring.value = std::vector<uint8_t>(n_bytes);

  // loop over all bytes
  for (int byte_idx = 0; byte_idx < n_bytes; byte_idx++) {

    // loop over bits in a byte
    for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {

      // map bit index in byte to bit index in total bitstring
      int bit_idx = bit_idx_in_byte + byte_idx * bits_per_byte;
      if ((byte_idx + 1) >= n_bytes && (bit_idx_in_byte + bitstring.bits_unused) >= bits_per_byte) break;

      // set bit in output bitstring appropriately
      bitstring.value[byte_idx] |= bits[bit_idx] << (bits_per_byte - 1 - bit_idx_in_byte);
    }
  }
}

#endif // ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_SETTERS_H