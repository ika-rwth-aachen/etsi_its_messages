#include <gtest/gtest.h>
#include <cmath>

namespace spatem_ts_access = etsi_its_spatem_ts_msgs::access;

TEST(etsi_its_spatem_ts_msgs, test_set_get_spatem) {

  spatem_ts_msgs::IntersectionState intsct;
  unsigned int id = randomInt(spatem_ts_msgs::IntersectionID::MIN, spatem_ts_msgs::IntersectionID::MAX);
  spatem_ts_access::setIntersectionID(intsct, id);
  EXPECT_EQ(id, spatem_ts_access::getIntersectionID(intsct));

  unsigned int moy = randomInt(spatem_ts_msgs::MinuteOfTheYear::MIN, spatem_ts_msgs::MinuteOfTheYear::MAX);
  spatem_ts_access::setMinuteOfTheYear(intsct, moy);
  EXPECT_EQ(moy, spatem_ts_access::getMinuteOfTheYear(intsct).value);
  EXPECT_EQ(true, intsct.moy_is_present);
  // Generate dummy time: 04.01.2007 1:15
  struct tm timeinfo;
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 15;
  timeinfo.tm_hour = 1;
  timeinfo.tm_mday = 4;
  timeinfo.tm_mon = 0;
  timeinfo.tm_year = 107; //years since 1900
  uint64_t unix_stamp = timegm(&timeinfo);
  // Set time to beginning of 2007: 01.01.2007 0:00
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 0;
  timeinfo.tm_hour = 0;
  timeinfo.tm_mday = 1;
  timeinfo.tm_mon = 0;
  timeinfo.tm_year = 107; //years since 1900
  EXPECT_EQ((timegm(&timeinfo)+60*moy)*1e9, spatem_ts_access::getUnixNanosecondsFromMinuteOfTheYear(spatem_ts_access::getMinuteOfTheYear(intsct), unix_stamp*1e9));

  unsigned int dsecond = randomInt(spatem_ts_msgs::DSecond::MIN, spatem_ts_msgs::DSecond::MAX);
  spatem_ts_access::setDSecond(intsct, dsecond);
  EXPECT_EQ(dsecond, spatem_ts_access::getDSecond(intsct).value);
  EXPECT_EQ(true, intsct.time_stamp_is_present);

  double dsecond_double = randomDouble(((double)spatem_ts_msgs::DSecond::MIN)*1e-3, ((double)spatem_ts_msgs::DSecond::MAX)*1e-3);
  spatem_ts_access::setDSecond(intsct, dsecond_double);
  EXPECT_NEAR(dsecond_double, spatem_ts_access::getDSecondValue(intsct), 1e-3);
  EXPECT_EQ(true, intsct.time_stamp_is_present);

  spatem_ts_msgs::MovementState movement_state;
  unsigned int signal_group_id = randomInt(spatem_ts_msgs::SignalGroupID::MIN, spatem_ts_msgs::SignalGroupID::MAX);
  spatem_ts_access::setSignalGroupID(movement_state, signal_group_id);
  EXPECT_EQ(signal_group_id, spatem_ts_access::getSignalGroupID(movement_state));
}