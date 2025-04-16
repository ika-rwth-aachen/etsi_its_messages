#include <gtest/gtest.h>
#include <cmath>

namespace denm_access = etsi_its_denm_msgs::access;

TEST(etsi_its_denm_msgs, test_set_get_denm) {
  denm_msgs::DENM denm;

  int station_id = randomInt(denm_msgs::StationID::MIN, denm_msgs::StationID::MAX);
  int protocol_version =
      randomInt(denm_msgs::ItsPduHeader::PROTOCOL_VERSION_MIN, denm_msgs::ItsPduHeader::PROTOCOL_VERSION_MAX);
  denm_access::setItsPduHeader(denm, station_id, protocol_version);
  EXPECT_EQ(denm_msgs::ItsPduHeader::MESSAGE_ID_DENM, denm.header.message_id);
  EXPECT_EQ(protocol_version, denm.header.protocol_version);
  EXPECT_EQ(station_id, denm_access::getStationID(denm));

  // https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/01.02.01_60/ts_10289402v010201p.pdf
  // DE_TimestampITS
  // The value for TimestampIts for 2007-01-01T00:00:00.000Z is
  // 94694401000 milliseconds, which includes one leap second insertion
  // since 2004-01-01T00:00:00.000Z.
  uint64_t t_2007 = ((uint64_t)1167609600) * 1e9;
  denm_msgs::TimestampIts t_its;
  denm_access::setTimestampITS(t_its, t_2007, 1);
  EXPECT_EQ(94694401000, t_its.value);

  denm_access::setReferenceTime(denm, t_2007, 1);
  EXPECT_EQ(94694401000, denm_access::getReferenceTimeValue(denm));
  uint64_t t_2007_off = t_2007 + 5 * 1e9;
  EXPECT_EQ(t_2007, denm_access::getUnixNanosecondsFromReferenceTime(denm_access::getReferenceTime(denm)));

  int stationType_val = randomInt(denm_msgs::StationType::MIN, denm_msgs::StationType::MAX);
  denm_access::setStationType(denm, stationType_val);
  EXPECT_EQ(stationType_val, denm_access::getStationType(denm));

  double latitude = randomDouble(-90.0, 90.0);
  double longitude = randomDouble(-180.0, 180.0);
  denm_access::setReferencePosition(denm, latitude, longitude);
  EXPECT_NEAR(latitude, denm_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_access::getLongitude(denm), 1e-7);
  latitude = randomDouble(-90.0, 90.0);
  longitude = randomDouble(-180.0, 180.0);
  double altitude = randomDouble(-1000.0, 8000.0);
  denm_access::setReferencePosition(denm, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, denm_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_access::getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, denm_access::getAltitude(denm), 1e-2);

  // Set specific position to test utm projection
  latitude = 50.787467;
  longitude = 6.046498;
  altitude = 209.0;
  denm_access::setReferencePosition(denm, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = denm_access::getUTMPosition(denm, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  denm_access::setFromUTMPosition(denm, utm, zone, northp);
  EXPECT_NEAR(latitude, denm_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_access::getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, denm_access::getAltitude(denm), 1e-2);

  double heading_val = randomDouble(0.0, 360.0);
  double heading_conf = randomDouble(0.0, 6.25);
  denm.denm.location_is_present = true;
  denm_access::setHeading(denm, heading_val, heading_conf);
  EXPECT_NEAR(heading_val, denm_access::getHeading(denm), 1e-1);
  EXPECT_NEAR(heading_conf, denm_access::getHeadingConfidence(denm), 1e-1);

  double speed_val = randomDouble(0.0, 163.82);
  denm.denm.location_is_present = true;
  denm_access::setSpeed(denm, speed_val);
  EXPECT_NEAR(speed_val, denm_access::getSpeed(denm), 1e-2);
}