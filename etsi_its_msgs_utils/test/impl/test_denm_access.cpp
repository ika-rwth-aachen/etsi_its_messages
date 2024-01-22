#include <cmath>
#include <limits>
#include <random>
#include <chrono>
#include <ctime>

#include <gtest/gtest.h>

std::default_random_engine random_engine;


double randomDouble(double min, double max) {
  std::uniform_real_distribution<double> uniform_distribution_double(min, max);
  return uniform_distribution_double(random_engine);
}

int randomInt(int min, int max) {
  std::uniform_int_distribution<int> uniform_distribution_int(min, max);
  return uniform_distribution_int(random_engine);
}

TEST(etsi_its_denm_msgs, test_set_get_denm) {

  DENM denm;

  int station_id = randomInt(StationID::MIN,StationID::MAX);
  int protocol_version = randomInt(ItsPduHeader::PROTOCOL_VERSION_MIN,ItsPduHeader::PROTOCOL_VERSION_MAX);
  setItsPduHeader(denm, station_id, protocol_version);
  EXPECT_EQ(ItsPduHeader::MESSAGE_ID_DENM, denm.header.message_id);
  EXPECT_EQ(protocol_version, denm.header.protocol_version);
  EXPECT_EQ(station_id, getStationID(denm));

  // https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/01.02.01_60/ts_10289402v010201p.pdf
  // DE_TimestampITS
  // The value for TimestampIts for 2007-01-01T00:00:00.000Z is
  // 94694401000 milliseconds, which includes one leap second insertion
  // since 2004-01-01T00:00:00.000Z.
  uint64_t t_2007 = ((uint64_t)1167609600)*1e9;
  TimestampIts t_its;
  etsi_its_msgs::cdd_access::setTimestampITS(t_its, t_2007, 1);
  EXPECT_EQ(94694401000, t_its.value);
  
  setReferenceTime(denm, t_2007, 1);
  EXPECT_EQ(94694401000, getReferenceTimeValue(denm));
  uint64_t t_2007_off = t_2007 + 5*1e9;
  EXPECT_EQ(t_2007, getUnixNanosecondsFromReferenceTime(getReferenceTime(denm), 1));

  int stationType_val = randomInt(StationType::MIN, StationType::MAX);
  setStationType(denm, stationType_val);
  EXPECT_EQ(stationType_val, getStationType(denm));

  double latitude = randomDouble(-90.0, 90.0);
  double longitude = randomDouble(-180.0, 180.0);
  setReferencePosition(denm, latitude, longitude);
  EXPECT_NEAR(latitude, getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(denm), 1e-7);
  latitude = randomDouble(-90.0, 90.0);
  longitude = randomDouble(-180.0, 180.0);
  double altitude = randomDouble(-1000.0, 8000.0);
  setReferencePosition(denm, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, getAltitude(denm), 1e-2);

  // Set specific position to test utm projection
  latitude = 50.787467;
  longitude = 6.046498;
  altitude = 209.0;
  setReferencePosition(denm, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = getUTMPosition(denm, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  setFromUTMPosition(denm, utm, zone, northp);
  EXPECT_NEAR(latitude, getLatitude(denm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(denm), 1e-7);
  EXPECT_NEAR(altitude, getAltitude(denm), 1e-2);

  double heading_val = randomDouble(0.0, 360.0);
  denm.denm.location_is_present = true;
  setHeading(denm, heading_val);
  EXPECT_NEAR(heading_val, getHeading(denm), 1e-1);

  double speed_val = randomDouble(0.0, 163.82);
  denm.denm.location_is_present = true;
  setSpeed(denm, speed_val);
  EXPECT_NEAR(speed_val, getSpeed(denm), 1e-2);
}
  
int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}