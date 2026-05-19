// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

/**
 * @file impl/constants.h
 * @brief File containing constants that are used in the context of ETIS ITS Messages
 */

#pragma once

#include <iostream>
#include <map>

namespace etsi_its_msgs {

const uint64_t UNIX_SECONDS_2004 = 1072915200;  // Unix-Seconds for 2004-01-01T00:00:00.000Z

/**
 * @brief std::map that stores all leap second insertions since 2004 with the corresponding unix-date of the insertion
 *
 */
const std::map<uint64_t, uint16_t> LEAP_SECOND_INSERTIONS_SINCE_2004{
    {UNIX_SECONDS_2004, 0},  // 2004-01-01T00:00:00.000Z
    {1136073599, 1},         // 2005-12-31T23:59:59.000Z
    {1230767999, 2},         // 2008-12-31T23:59:59.000Z
    {1341100799, 3},         // 2012-06-30T23:59:59.000Z
    {1435708799, 4},         // 2015-06-30T23:59:59.000Z
    {1483228799, 5}          // 2016-12-31T23:59:59.000Z
};

/**
 * @brief Get the leap second insertions since 2004 for given unix seconds
 *
 * @param unix_seconds the current unix seconds for that the leap second insertions since 2004 shall be provided
 * @return uint16_t the number of leap second insertions since 2004 for unix_seconds
 */
inline uint16_t getLeapSecondInsertionsSince2004(const uint64_t unix_seconds) {
  // Check if the map is empty
  if (LEAP_SECOND_INSERTIONS_SINCE_2004.empty()) return 0;
  auto it = LEAP_SECOND_INSERTIONS_SINCE_2004.upper_bound(
      unix_seconds);  // Find the first element greater than givenUnixSecond
  if (it == LEAP_SECOND_INSERTIONS_SINCE_2004.begin()) return 0;
  --it;               // Move iterator to the element with a key less than or equal to givenUnixSecond
  return it->second;  // Return the corresponding value
}

// An interval containing 95% of the points in a 1D Gaussian distribution
constexpr const double ONE_D_GAUSSIAN_FACTOR = 2.0;

// An ellipse containing 95% of the points in a 2D Gaussian distribution
// has size 2.4477*sigma_{major/minor}
constexpr const double TWO_D_GAUSSIAN_FACTOR = 2.4477;


// Helper struct to get the value of ONE_CENTIMETER from a given type, if it exists
// Otherwise, it defaults to 1
template <typename SemiAxisLength, typename = std::void_t<>>
struct OneCentimeterHelper {
    static constexpr int value = 1; // Default value
};

// Specialization for types that have ONE_CENTIMETER defined
template <typename SemiAxisLength>
struct OneCentimeterHelper<SemiAxisLength, std::void_t<decltype(SemiAxisLength::ONE_CENTIMETER)>> {
    static constexpr int value = SemiAxisLength::ONE_CENTIMETER;
};


}  // namespace etsi_its_msgs