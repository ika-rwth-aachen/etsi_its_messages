/*
=============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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

/**
 * @file impl/spatem/spatem_ts_getters.h
 * @brief Getter functions for the ETSI ITS SPATEM
 */

#pragma once

namespace etsi_its_spatem_ts_msgs {

namespace access {

#include <etsi_its_msgs_utils/impl/asn1_primitives/asn1_primitives_getters.h>
#include <etsi_its_msgs_utils/impl/checks.h>

/**
 * @brief Get the intersection-id
 * 
 * @param intsct_id intersection-id object to get the value from
 * @return uint16_t id of the intersection
 */
inline uint16_t getIntersectionID(const IntersectionID& intsct_id) {
  return intsct_id.value;
}

/**
 * @brief Get the intersection-id of an IntersectionReferenceID object
 * 
 * @param intsct_ref_id IntersectionReferenceID object
 * @return uint16_t id of the intersection
 */
inline uint16_t getIntersectionID(const IntersectionReferenceID& intsct_ref_id) {
  return getIntersectionID(intsct_ref_id.id);
}

/**
 * @brief Get the intersection-id of an IntersectionState object
 * 
 * @param intsct IntersectionState object
 * @return uint16_t id of the intersection
 */
inline uint16_t getIntersectionID(const IntersectionState& intsct) {
  return getIntersectionID(intsct.id);
}

} // namespace access

} // namespace etsi_its_spatem_ts_msgs
