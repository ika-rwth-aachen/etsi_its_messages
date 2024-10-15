#include <gtest/gtest.h>
#include <cmath>

namespace cpm_ts_access = etsi_its_cpm_ts_msgs::access;

TEST(etsi_its_cpm_ts_msgs, test_set_get_cpm) {
  cpm_ts_msgs::CollectivePerceptionMessage cpm;

  int station_id = randomInt(cpm_ts_msgs::StationId::MIN, cpm_ts_msgs::StationId::MAX);
  int protocol_version = randomInt(cpm_ts_msgs::OrdinalNumber1B::MIN, cpm_ts_msgs::OrdinalNumber1B::MAX);
  cpm_ts_access::setItsPduHeader(cpm, station_id, protocol_version);
  EXPECT_EQ(cpm_ts_msgs::MessageId::CPM, cpm.header.message_id.value);
  EXPECT_EQ(protocol_version, cpm.header.protocol_version.value);
  EXPECT_EQ(station_id, cpm_ts_access::getStationID(cpm));

  // https://www.etsi.org/deliver/etsi_ts/102800_102899/10289402/01.02.01_60/ts_10289402v010201p.pdf
  // DE_TimestampITS
  // The value for TimestampIts for 2007-01-01T00:00:00.000Z is
  // 94694401000 milliseconds, which includes one leap second insertion
  // since 2004-01-01T00:00:00.000Z.

  uint64_t t_2007 = ((uint64_t)1167609600) * 1e9;
  cpm_ts_msgs::TimestampIts t_its;
  EXPECT_EQ(1, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007 * 1e-9));
  cpm_ts_access::setTimestampITS(t_its, t_2007, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007 * 1e-9));
  EXPECT_EQ(94694401000, t_its.value);
  cpm_ts_access::setReferenceTime(cpm, t_2007, etsi_its_msgs::getLeapSecondInsertionsSince2004(t_2007 * 1e-9));
  EXPECT_EQ(94694401000, cpm_ts_access::getReferenceTimeValue(cpm));
  EXPECT_EQ(t_2007, cpm_ts_access::getUnixNanosecondsFromReferenceTime(cpm_ts_access::getReferenceTime(cpm)));

  double latitude = randomDouble(-90.0, 90.0);
  double longitude = randomDouble(-180.0, 180.0);
  cpm_ts_access::setReferencePosition(cpm, latitude, longitude);
  EXPECT_NEAR(latitude, cpm_ts_access::getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, cpm_ts_access::getLongitude(cpm), 1e-7);
  latitude = randomDouble(-90.0, 90.0);
  longitude = randomDouble(-180.0, 180.0);
  double altitude = randomDouble(-1000.0, 8000.0);
  cpm_ts_access::setReferencePosition(cpm, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, cpm_ts_access::getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, cpm_ts_access::getLongitude(cpm), 1e-7);
  EXPECT_NEAR(altitude, cpm_ts_access::getAltitude(cpm), 1e-2);

  // Set specific position to test utm projection
  latitude = 50.787467;
  longitude = 6.046498;
  altitude = 209.0;
  cpm_ts_access::setReferencePosition(cpm, latitude, longitude, altitude);
  int zone;
  bool northp;
  gm::PointStamped utm = cpm_ts_access::getUTMPosition(cpm, zone, northp);
  EXPECT_NEAR(291827.02, utm.point.x, 1e-1);
  EXPECT_NEAR(5630349.72, utm.point.y, 1e-1);
  EXPECT_EQ(altitude, utm.point.z);
  EXPECT_EQ(32, zone);
  EXPECT_EQ(true, northp);
  cpm_ts_access::setFromUTMPosition(cpm, utm, zone, northp);
  EXPECT_NEAR(latitude, cpm_ts_access::getLatitude(cpm), 1e-7);
  EXPECT_NEAR(longitude, cpm_ts_access::getLongitude(cpm), 1e-7);
  EXPECT_NEAR(altitude, cpm_ts_access::getAltitude(cpm), 1e-2);

  cpm_ts_msgs::PerceivedObject object;
  gm::Vector3 dimensions;
  dimensions.x = randomDouble(0.1, 25.6);
  dimensions.y = randomDouble(0.1, 25.6);
  dimensions.z = randomDouble(0.1, 25.6);
  cpm_ts_access::setDimensionsOfPerceivedObject(object, dimensions);
  EXPECT_NEAR(dimensions.x, cpm_ts_access::getDimensionsOfPerceivedObject(object).x, 1e-1);
  EXPECT_NEAR(dimensions.y, cpm_ts_access::getDimensionsOfPerceivedObject(object).y, 1e-1);
  EXPECT_NEAR(dimensions.z, cpm_ts_access::getDimensionsOfPerceivedObject(object).z, 1e-1);

  gm::Vector3 velocity;
  velocity.x = randomDouble(-163.83, 163.83);
  velocity.y = randomDouble(-163.83, 163.83);
  velocity.z = randomDouble(-163.83, 163.83);
  cpm_ts_access::setVelocityOfPerceivedObject(object, velocity);
  EXPECT_NEAR(velocity.x, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).x, 1e-2);
  EXPECT_NEAR(velocity.y, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).y, 1e-2);
  EXPECT_NEAR(velocity.z, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).z, 1e-2);

  gm::Vector3 acceleration;
  acceleration.x = randomDouble(-16.0, 16.0);
  acceleration.y = randomDouble(-16.0, 16.0);
  acceleration.z = randomDouble(-16.0, 16.0);
  cpm_ts_access::setAccelerationOfPerceivedObject(object, acceleration);
  EXPECT_NEAR(acceleration.x, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).x, 1e-1);
  EXPECT_NEAR(acceleration.y, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).y, 1e-1);
  EXPECT_NEAR(acceleration.z, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).z, 1e-1);
}