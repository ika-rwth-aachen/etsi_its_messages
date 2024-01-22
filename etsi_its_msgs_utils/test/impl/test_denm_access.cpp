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
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}