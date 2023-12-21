#pragma once

#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <ctime>

#include <gtest/gtest.h>

#include "test_helpers.h"
#include "test_j2735_access.h"


TEST(etsi_its_mapem_msgs, test_set_get_mapem) {

  TestHelper helper;

  MAPEM mapem;

  unsigned int moy = helper.randomInt(MinuteOfTheYear::MIN,MinuteOfTheYear::MAX);
  setMinuteOfTheYear(mapem, moy);
  EXPECT_EQ(moy, getMinuteOfTheYearValue(mapem));
  // Generate dummy time: 04.01.2007 1:15
  struct tm timeinfo;
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 15;
  timeinfo.tm_hour = 1;
  timeinfo.tm_mday = 4;
  timeinfo.tm_mon = 0;
  timeinfo.tm_year = 107; //years since 1900
  uint64_t unix_stamp = mktime(&timeinfo);
  // Set time to beginning of 2007: 01.01.2007 0:00
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 0;
  timeinfo.tm_hour = 0;
  timeinfo.tm_mday = 1;
  timeinfo.tm_mon = 0;
  timeinfo.tm_year = 107; //years since 1900
  EXPECT_EQ((mktime(&timeinfo)+60*moy)*1e9, getUnixNanoseconds(mapem, unix_stamp*1e9));
  
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}