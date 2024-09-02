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

TEST(etsi_its_cpm_ts_msgs, test_set_get_cpm) {

  CollectivePerceptionMessage cpm;

  int station_id = randomInt(StationId::MIN,StationId::MAX);
  int protocol_version = randomInt(OrdinalNumber1B::MIN,OrdinalNumber1B::MAX);
  setItsPduHeader(cpm, station_id, protocol_version);
  EXPECT_EQ(MessageId::CPM, cpm.header.message_id.value);
  EXPECT_EQ(protocol_version, cpm.header.protocol_version.value);
  EXPECT_EQ(station_id, getStationID(cpm));

  // https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/01.02.01_60/ts_10289402v010201p.pdf
  // DE_TimestampITS
  // The value for TimestampIts for 2007-01-01T00:00:00.000Z is
  // 94694401000 milliseconds, which includes one leap second insertion
  // since 2004-01-01T00:00:00.000Z.
  
  uint64_t t_2007 = ((uint64_t)1167609600)*1e9;
  TimestampIts t_its;
  EXPECT_EQ(1, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007*1e-9));
  etsi_its_cpm_ts_msgs::access::setTimestampITS(t_its, t_2007, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007*1e-9));
  EXPECT_EQ(94694401000, t_its.value);
  setReferenceTime(cpm, t_2007, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007*1e-9));
  EXPECT_EQ(94694401000, getReferenceTimeValue(cpm));
  EXPECT_EQ(t_2007, getUnixNanosecondsFromReferenceTime(getReferenceTime(cpm), 1));

  double latitude = randomDouble(-90.0, 90.0);
  double longitude = randomDouble(-180.0, 180.0);
  setReferencePosition(cpm, latitude, longitude);
  EXPECT_NEAR(latitude, getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(cpm), 1e-7);
  latitude = randomDouble(-90.0, 90.0);
  longitude = randomDouble(-180.0, 180.0);
  double altitude = randomDouble(-1000.0, 8000.0);
  setReferencePosition(cpm, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(cpm), 1e-7);
  EXPECT_NEAR(altitude, getAltitude(cpm), 1e-2);

  // Set specific position to test utm projection
  latitude = 50.787467;
  longitude = 6.046498;
  altitude = 209.0;
  setReferencePosition(cpm, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = getUTMPosition(cpm, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  setFromUTMPosition(cpm, utm, zone, northp);
  EXPECT_NEAR(latitude, getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, getLongitude(cpm), 1e-7);
  EXPECT_NEAR(altitude, getAltitude(cpm), 1e-2);

  // TODO: check object list access
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
