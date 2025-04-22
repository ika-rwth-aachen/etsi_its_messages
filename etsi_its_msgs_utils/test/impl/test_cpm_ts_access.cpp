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

  // Set WGS84 confidence ellipse
  std::array<double, 4> covariance_matrix = {randomDouble(1.0, 100.0), 0.0,
    0.0, randomDouble(1.0, 100.0)};
  double phi = randomDouble(0.0, M_PI_2);
  std::array<double, 4> covariance_matrix_rotated = {
    covariance_matrix[0] * std::cos(phi) * std::cos(phi) +
    covariance_matrix[3] * std::sin(phi) * std::sin(phi),
    (covariance_matrix[0] - covariance_matrix[3]) * std::cos(phi) * std::sin(phi),
    (covariance_matrix[0] - covariance_matrix[3]) * std::cos(phi) * std::sin(phi),
    covariance_matrix[0] * std::sin(phi) * std::sin(phi) +
    covariance_matrix[3] * std::cos(phi) * std::cos(phi)
  };
  cpm_ts_access::setWGSRefPosConfidence(cpm, covariance_matrix_rotated);
  std::array<double, 4> cov_get_rotated = cpm_ts_access::getWGSRefPosConfidence(cpm);
  EXPECT_NEAR(covariance_matrix_rotated[0], cov_get_rotated[0], 1e-1);
  EXPECT_NEAR(covariance_matrix_rotated[1], cov_get_rotated[1], 1e-1);
  EXPECT_NEAR(covariance_matrix_rotated[2], cov_get_rotated[2], 1e-1);
  EXPECT_NEAR(covariance_matrix_rotated[3], cov_get_rotated[3], 1e-1);

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

  // Test perceived object
  // Position
  cpm_ts_msgs::PerceivedObject object;
  double dx = randomDouble(-10.0, 10.0);
  double dy = randomDouble(-10.0, 10.0);
  double dz = randomDouble(-10.0, 10.0);
  utm.point.x += dx;
  utm.point.y += dy;
  utm.point.z += dz;
  double var_x = randomDouble(0.1, 20.48);
  double var_y = randomDouble(0.1, 20.48);
  double var_z = randomDouble(0.1, 20.48);
  cpm_ts_access::setUTMPositionOfPerceivedObject(cpm, object, utm, var_x, var_y, var_z);
  gm::PointStamped point = cpm_ts_access::getUTMPositionOfPerceivedObject(cpm, object);
  EXPECT_NEAR(utm.point.x, point.point.x, 1e-1);
  EXPECT_NEAR(utm.point.y, point.point.y, 1e-1);
  EXPECT_NEAR(utm.point.z, point.point.z, 1e-1);
  gm::Point dpoint = cpm_ts_access::getPositionOfPerceivedObject(object);
  EXPECT_NEAR(dx, dpoint.x, 1e-1);
  EXPECT_NEAR(dy, dpoint.y, 1e-1);
  EXPECT_NEAR(dz, dpoint.z, 1e-1);
  auto [var_x_get, var_y_get, var_z_get] = cpm_ts_access::getPositionConfidenceOfPerceivedObject(object);
  EXPECT_NEAR(var_x, var_x_get, 1e-2);
  EXPECT_NEAR(var_y, var_y_get, 1e-2);
  EXPECT_NEAR(var_z, var_z_get, 1e-2);

  // Dimensions
  gm::Vector3 dimensions;
  dimensions.x = randomDouble(0.1, 25.6);
  dimensions.y = randomDouble(0.1, 25.6);
  dimensions.z = randomDouble(0.1, 25.6);
  var_x = randomDouble(0.1, 1.5);
  var_y = randomDouble(0.1, 1.5);
  var_z = randomDouble(0.1, 1.5);
  cpm_ts_access::setDimensionsOfPerceivedObject(object, dimensions, var_x, var_y, var_z);
  EXPECT_NEAR(dimensions.x, cpm_ts_access::getDimensionsOfPerceivedObject(object).x, 1e-1);
  EXPECT_NEAR(dimensions.y, cpm_ts_access::getDimensionsOfPerceivedObject(object).y, 1e-1);
  EXPECT_NEAR(dimensions.z, cpm_ts_access::getDimensionsOfPerceivedObject(object).z, 1e-1);
  std::tie(var_x_get, var_y_get, var_z_get) = cpm_ts_access::getDimensionsConfidenceOfPerceivedObject(object);
  EXPECT_NEAR(var_x, var_x_get, 1e-1);
  EXPECT_NEAR(var_y, var_y_get, 1e-1);
  EXPECT_NEAR(var_z, var_z_get, 1e-1);

  // Yaw
  double yaw = randomDouble(-M_PI, M_PI);
  double yaw_std = randomDouble(0.0001, 0.109);
  cpm_ts_access::setYawOfPerceivedObject(object, yaw, yaw_std);
  EXPECT_NEAR(yaw, cpm_ts_access::getYawOfPerceivedObject(object), 1e-1 * M_PI / 180.0);
  EXPECT_NEAR(yaw_std, cpm_ts_access::getYawConfidenceOfPerceivedObject(object), 1e-1 * M_PI / 180.0);

  // Yaw rate
  double yaw_rate = randomDouble(-4.45, 4.45);
  double yaw_rate_std = randomDouble(0.0001, 0.436);
  cpm_ts_access::setYawRateOfPerceivedObject(object, yaw_rate, yaw_rate_std);
  EXPECT_NEAR(yaw_rate, cpm_ts_access::getYawRateOfPerceivedObject(object), 1e0 * M_PI / 180.0);
  std::array<double, 6> yaw_std_possible_values{1.0, 2.0, 5.0, 10.0, 20.0, 50.0};
  std::for_each(yaw_std_possible_values.begin(), yaw_std_possible_values.end(),
                [](double& val) { val *= 0.5 * M_PI / 180.0; });
  double expected_yaw_rate = *std::lower_bound(yaw_std_possible_values.begin(), yaw_std_possible_values.end(), yaw_rate_std);
  EXPECT_NEAR(expected_yaw_rate, cpm_ts_access::getYawRateConfidenceOfPerceivedObject(object), 1e0 * M_PI / 180.0);

  // Velocity
  gm::Vector3 velocity;
  velocity.x = randomDouble(-163.83, 163.83);
  velocity.y = randomDouble(-163.83, 163.83);
  velocity.z = randomDouble(-163.83, 163.83);
  double velocity_x_std = randomDouble(0.01, 0.625);
  double velocity_y_std = randomDouble(0.01, 0.625);
  double velocity_z_std = randomDouble(0.01, 0.625);
  cpm_ts_access::setVelocityOfPerceivedObject(object, velocity, velocity_x_std, velocity_y_std, velocity_z_std);
  EXPECT_NEAR(velocity.x, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).x, 1e-2);
  EXPECT_NEAR(velocity.y, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).y, 1e-2);
  EXPECT_NEAR(velocity.z, cpm_ts_access::getCartesianVelocityOfPerceivedObject(object).z, 1e-2);
  auto [vel_x_std_get, vel_y_std_get, vel_z_std_get] = cpm_ts_access::getCartesianVelocityConfidenceOfPerceivedObject(object);
  EXPECT_NEAR(velocity_x_std, vel_x_std_get, 1e-2);
  EXPECT_NEAR(velocity_y_std, vel_y_std_get, 1e-2);
  EXPECT_NEAR(velocity_z_std, vel_z_std_get, 1e-2);

  // Acceleration
  gm::Vector3 acceleration;
  acceleration.x = randomDouble(-16.0, 16.0);
  acceleration.y = randomDouble(-16.0, 16.0);
  acceleration.z = randomDouble(-16.0, 16.0);
  double acceleration_x_std = randomDouble(0.1, 5.0);
  double acceleration_y_std = randomDouble(0.1, 5.0);
  double acceleration_z_std = randomDouble(0.1, 5.0);
  cpm_ts_access::setAccelerationOfPerceivedObject(object, acceleration, acceleration_x_std, acceleration_y_std, acceleration_z_std);
  EXPECT_NEAR(acceleration.x, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).x, 1e-1);
  EXPECT_NEAR(acceleration.y, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).y, 1e-1);
  EXPECT_NEAR(acceleration.z, cpm_ts_access::getCartesianAccelerationOfPerceivedObject(object).z, 1e-1);
  auto [acc_x_std_get, acc_y_std_get, acc_z_std_get] = cpm_ts_access::getCartesianAccelerationConfidenceOfPerceivedObject(object);
  EXPECT_NEAR(acceleration_x_std, acc_x_std_get, 1e-1);
  EXPECT_NEAR(acceleration_y_std, acc_y_std_get, 1e-1);
  EXPECT_NEAR(acceleration_z_std, acc_z_std_get, 1e-1);

  // SensorInformation
  cpm_ts_msgs::WrappedCpmContainer sensor_information_container;
  cpm_ts_access::initSensorInformationContainer(sensor_information_container);
  int sensor_id = randomInt(cpm_ts_msgs::Identifier1B::MIN, cpm_ts_msgs::Identifier1B::MAX);
  int sensor_type = randomInt(cpm_ts_msgs::SensorType::MIN, cpm_ts_msgs::SensorType::MAX);
  cpm_ts_msgs::SensorInformation sensor_information;
  cpm_ts_access::setSensorID(sensor_information, sensor_id);
  cpm_ts_access::setSensorType(sensor_information, sensor_type);
  cpm_ts_access::addSensorInformationToContainer(sensor_information_container, sensor_information);
  EXPECT_EQ(sensor_id, cpm_ts_access::getSensorID(sensor_information));
  EXPECT_EQ(sensor_type, cpm_ts_access::getSensorType(sensor_information));
}