// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

#ifndef ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_GETTERS_H
#define ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_ASN1_PRIMITIVES_GETTERS_H

/**
 * @file impl/asn1_primitives_getters.h
 * @brief File containing getter-functions that are used in the context of ETIS ITS Messages related to ASN.1 datatype primitives
 */

/**
 * @brief Get a Bit String in form of bool vector
 *
 * @param buffer as uint8_t vector
 * @param bits_unused number of bits to ignore at the end of the bit string
 * @return std::vector<bool>
 */
inline std::vector<bool> getBitString(const std::vector<uint8_t>& buffer, const int bits_unused) {

  // bit string size
  const int bits_per_byte = 8;
  const int n_bytes = buffer.size();
  const int n_bits = n_bytes * bits_per_byte;
  std::vector<bool> bits;
  bits.resize(n_bits - bits_unused, 0);

  // loop over bytes
  for (int byte_idx = 0; byte_idx < n_bytes; byte_idx++) {

    // loop over bits in a byte
    for (int bit_idx_in_byte = 0; bit_idx_in_byte < bits_per_byte; bit_idx_in_byte++) {

      // map bit index in byte to bit index in total bitstring
      int bit_idx = bit_idx_in_byte + byte_idx * bits_per_byte;
      if ((byte_idx + 1) >= n_bytes && (bit_idx_in_byte + bits_unused) >= bits_per_byte) break;

      // extract bit from bitstring and set output array entry appropriately
      bool byte_has_true_bit = static_cast<bool>(buffer[byte_idx] & (1 << (bits_per_byte - 1 - bit_idx_in_byte)));
      if (byte_has_true_bit) bits[bit_idx] = true;
    }
  }
  return bits;
}

#endif  // ETSI_ITS_MSGS_UTILS_IMPL_ASN1_PRIMITIVES_GETTERS_H