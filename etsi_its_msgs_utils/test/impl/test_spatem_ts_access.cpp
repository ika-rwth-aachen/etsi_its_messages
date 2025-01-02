#include <gtest/gtest.h>
#include <cmath>

namespace spatem_ts_access = etsi_its_spatem_ts_msgs::access;

TEST(etsi_its_spatem_ts_msgs, test_set_get_spatem) {

  unsigned int moy = randomInt(spatem_ts_msgs::MinuteOfTheYear::MIN, spatem_ts_msgs::MinuteOfTheYear::MAX);
  spatem_ts_msgs::MinuteOfTheYear minute_of_the_year;
  minute_of_the_year.value = moy;
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
  EXPECT_EQ((timegm(&timeinfo)+60*moy)*1e9, spatem_ts_access::getUnixNanosecondsFromMinuteOfTheYear(minute_of_the_year, unix_stamp*1e9));
}