#include <gtest/gtest.h>
#include <cmath>

namespace mapem_ts_access = etsi_its_mapem_ts_msgs::access;

TEST(etsi_its_mapem_ts_msgs, test_set_get_mapem) {
  mapem_ts_msgs::IntersectionGeometry intsct;

  unsigned int intersection_id = randomInt(mapem_ts_msgs::IntersectionID::MIN, mapem_ts_msgs::IntersectionID::MAX);
  mapem_ts_access::setIntersectionID(intsct, intersection_id);
  EXPECT_EQ(intersection_id, mapem_ts_access::getIntersectionID(intsct));

  // Set specific position to test utm projection
  double latitude = 50.787467;
  double longitude = 6.046498;
  double altitude = 209.0;
  mapem_ts_access::setPosition3D(intsct, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = mapem_ts_access::getRefPointUTMPosition(intsct, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  mapem_ts_access::setPosition3DFromUTMPosition(intsct.ref_point, utm, zone, northp);
  EXPECT_NEAR(latitude, mapem_ts_access::getLatitude(intsct.ref_point), 1e-7);
  EXPECT_NEAR(longitude, mapem_ts_access::getLongitude(intsct.ref_point), 1e-7);
  EXPECT_NEAR(altitude, mapem_ts_access::getElevation(intsct.ref_point), 1e-2);

  mapem_ts_msgs::MAPEM mapem;

  unsigned int moy = randomInt(mapem_ts_msgs::MinuteOfTheYear::MIN, mapem_ts_msgs::MinuteOfTheYear::MAX);
  mapem_ts_access::setMinuteOfTheYear(mapem, moy);
  EXPECT_EQ(moy, mapem_ts_access::getMinuteOfTheYearValue(mapem));
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
  EXPECT_EQ((timegm(&timeinfo)+60*moy)*1e9, mapem_ts_access::getUnixNanoseconds(mapem, unix_stamp*1e9));
}