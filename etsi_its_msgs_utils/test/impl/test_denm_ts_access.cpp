#include <gtest/gtest.h>
#include <cmath>

namespace denm_ts_access = etsi_its_denm_ts_msgs::access;

TEST(etsi_its_denm_ts_msgs, test_set_get_denm) {
  denm_ts_msgs::DENM denm;

  int station_id = randomInt(denm_ts_msgs::StationId::MIN, denm_ts_msgs::StationId::MAX);
  int protocol_version = randomInt(denm_ts_msgs::OrdinalNumber1B::MIN, denm_ts_msgs::OrdinalNumber1B::MAX);
  denm_ts_access::setItsPduHeader(denm, station_id, protocol_version);
  EXPECT_EQ(denm_ts_msgs::MessageId::DENM, denm.header.message_id.value);
  EXPECT_EQ(protocol_version, denm.header.protocol_version.value);
  EXPECT_EQ(station_id, denm_ts_access::getStationID(denm));

  // https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/01.02.01_60/ts_10289402v010201p.pdf
  // DE_TimestampITS
  // The value for TimestampIts for 2007-01-01T00:00:00.000Z is
  // 94694401000 milliseconds, which includes one leap second insertion
  // since 2004-01-01T00:00:00.000Z.
  uint64_t t_2007 = ((uint64_t)1167609600) * 1e9;
  denm_ts_msgs::TimestampIts t_its;
  denm_ts_access::setTimestampITS(t_its, t_2007, 1);
  EXPECT_EQ(94694401000, t_its.value);

  denm_ts_access::setReferenceTime(denm, t_2007, 1);
  EXPECT_EQ(94694401000, denm_ts_access::getReferenceTimeValue(denm));
  uint64_t t_2007_off = t_2007 + 5 * 1e9;
  EXPECT_EQ(t_2007, denm_ts_access::getUnixNanosecondsFromReferenceTime(denm_ts_access::getReferenceTime(denm)));

  int stationType_val = randomInt(denm_ts_msgs::StationType::MIN, denm_ts_msgs::StationType::MAX);
  denm_ts_access::setStationType(denm, stationType_val);
  EXPECT_EQ(stationType_val, denm_ts_access::getStationType(denm));

  double latitude = randomDouble(-90.0, 90.0);
  double longitude = randomDouble(-180.0, 180.0);
  denm_ts_access::setReferencePosition(denm, latitude, longitude);
  EXPECT_NEAR(latitude, denm_ts_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_ts_access::getLongitude(denm), 1e-7);
  latitude = randomDouble(-90.0, 90.0);
  longitude = randomDouble(-180.0, 180.0);
  double altitude = randomDouble(-1000.0, 8000.0);
  denm_ts_access::setReferencePosition(denm, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, denm_ts_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_ts_access::getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, denm_ts_access::getAltitude(denm), 1e-2);

  // Set specific position to test utm projection
  latitude = 50.787467;
  longitude = 6.046498;
  altitude = 209.0;
  denm_ts_access::setReferencePosition(denm, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = denm_ts_access::getUTMPosition(denm, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  denm_ts_access::setFromUTMPosition(denm, utm, zone, northp);
  EXPECT_NEAR(latitude, denm_ts_access::getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, denm_ts_access::getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, denm_ts_access::getAltitude(denm), 1e-2);

  double heading_val = randomDouble(0.0, 360.0);
  double heading_conf = randomDouble(0.0, 6.25);
  denm.denm.location_is_present = true;
  denm_ts_access::setWGSHeading(denm, heading_val, heading_conf);
  EXPECT_NEAR(heading_val, denm_ts_access::getWGSHeading(denm), 1e-1);
  EXPECT_NEAR(heading_conf, denm_ts_access::getWGSHeadingConfidence(denm), 1e-1);

  double speed_val = randomDouble(0.0, 163.82);
  denm.denm.location_is_present = true;
  denm_ts_access::setSpeed(denm, speed_val);
  EXPECT_NEAR(speed_val, denm_ts_access::getSpeed(denm), 1e-2);

  // (Sub)CauseCode exemplary test
  denm.denm.situation_is_present = true;
  denm.denm.situation.event_type.cc_and_scc.choice = denm_ts_msgs::CauseCodeChoice().CHOICE_TRAFFIC_CONDITION1;
  denm.denm.situation.event_type.cc_and_scc.traffic_condition1.value = denm_ts_msgs::TrafficConditionSubCauseCode().TRAFFIC_JAM_SLOWLY_INCREASING;
  EXPECT_EQ(denm_ts_msgs::CauseCodeChoice().CHOICE_TRAFFIC_CONDITION1, denm_ts_access::getCauseCode(denm));
  EXPECT_EQ(denm_ts_msgs::TrafficConditionSubCauseCode().TRAFFIC_JAM_SLOWLY_INCREASING, denm_ts_access::getSubCauseCode(denm));
  EXPECT_EQ("traffic condition", denm_ts_access::getCauseCodeType(denm));
  EXPECT_EQ("traffic jam slowly increasing", denm_ts_access::getSubCauseCodeType(denm));
}