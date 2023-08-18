/**
 * @file
 * @brief Sanity-check functions etc. for the ETSI ITS Common Data Dictionary (CDD)
 */

#pragma once

namespace etsi_its_msgs {

namespace cdd_access {

  template <typename T1, typename T2>
  void throwIfOutOfRange(const T1& val, const T2& min, const T2& max, std::string val_desc) {
    if (val < min || val > max) throw std::invalid_argument(val_desc+" value is out of range ("+std::to_string(min)+"..."+std::to_string(max)+")!");
  }

} // namespace cdd_access

} // namespace etsi_its_msgs