#include <gtest/gtest.h>
#include <cmath>

namespace spatem_ts_access = etsi_its_spatem_ts_msgs::access;

TEST(etsi_its_spatem_ts_msgs, test_set_get_spatem) {

  spatem_ts_msgs::IntersectionState intsct;
  unsigned int id = randomInt(spatem_ts_msgs::IntersectionID::MIN, spatem_ts_msgs::IntersectionID::MAX);
  spatem_ts_access::setIntersectionID(intsct, id);
  EXPECT_EQ(id, spatem_ts_access::getIntersectionID(intsct));

  unsigned int moy = randomInt(spatem_ts_msgs::MinuteOfTheYear::MIN, spatem_ts_msgs::MinuteOfTheYear::MAX);
  spatem_ts_access::setMinuteOfTheYear(intsct, moy);
  EXPECT_EQ(moy, spatem_ts_access::getMinuteOfTheYear(intsct).value);
  EXPECT_EQ(true, intsct.moy_is_present);
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
  EXPECT_EQ((timegm(&timeinfo)+60*moy)*1e9, spatem_ts_access::getUnixNanosecondsFromMinuteOfTheYear(spatem_ts_access::getMinuteOfTheYear(intsct), unix_stamp*1e9));

  unsigned int dsecond = randomInt(spatem_ts_msgs::DSecond::MIN, spatem_ts_msgs::DSecond::MAX);
  spatem_ts_access::setDSecond(intsct, dsecond);
  EXPECT_EQ(dsecond, spatem_ts_access::getDSecond(intsct).value);
  EXPECT_EQ(true, intsct.time_stamp_is_present);

  double dsecond_double = randomDouble(((double)spatem_ts_msgs::DSecond::MIN)*1e-3, ((double)spatem_ts_msgs::DSecond::MAX)*1e-3);
  spatem_ts_access::setDSecond(intsct, dsecond_double);
  EXPECT_NEAR(dsecond_double, spatem_ts_access::getDSecondValue(intsct), 1e-3);
  EXPECT_EQ(true, intsct.time_stamp_is_present);

  spatem_ts_msgs::MovementState movement_state;
  unsigned int signal_group_id = randomInt(spatem_ts_msgs::SignalGroupID::MIN, spatem_ts_msgs::SignalGroupID::MAX);
  spatem_ts_access::setSignalGroupID(movement_state, signal_group_id);
  EXPECT_EQ(signal_group_id, spatem_ts_access::getSignalGroupID(movement_state));
  movement_state.state_time_speed.array.resize(1);
  unsigned int event_state = randomInt(spatem_ts_msgs::MovementPhaseState::UNAVAILABLE, spatem_ts_msgs::MovementPhaseState::CAUTION_CONFLICTING_TRAFFIC);
  movement_state.state_time_speed.array[0].event_state.value = event_state;
  EXPECT_EQ(event_state, spatem_ts_access::getCurrentMovementPhaseStateValue(movement_state));

  std::array<float, 4> movement_phase_state_color = spatem_ts_access::interpretMovementPhaseStateAsColor(5);
  EXPECT_EQ(movement_phase_state_color[0], 0.18f); // color green
  EXPECT_EQ(movement_phase_state_color[1], 0.79f);
  EXPECT_EQ(movement_phase_state_color[2], 0.21f);
  EXPECT_EQ(movement_phase_state_color[3], 1.0f);

  std::array<float, 4> movement_phase_state_color2 = spatem_ts_access::interpretMovementPhaseStateAsColor(9);
  EXPECT_EQ(movement_phase_state_color2[0], 0.9f); // color orange
  EXPECT_EQ(movement_phase_state_color2[1], 0.7f);
  EXPECT_EQ(movement_phase_state_color2[2], 0.09f);
  EXPECT_EQ(movement_phase_state_color2[3], 1.0f);

  std::array<float, 4> movement_phase_state_color3 = spatem_ts_access::interpretMovementPhaseStateAsColor(10);
  EXPECT_EQ(movement_phase_state_color3[0], 0.5f); // color grey (out of definition range)
  EXPECT_EQ(movement_phase_state_color3[1], 0.5f);
  EXPECT_EQ(movement_phase_state_color3[2], 0.5f);
  EXPECT_EQ(movement_phase_state_color3[3], 1.0f);

  float confidence_as_float = spatem_ts_access::interpretTimeIntervalConfidenceAsFloat(0);
  EXPECT_EQ(confidence_as_float, 0.21f);

  float confidence_as_float2 = spatem_ts_access::interpretTimeIntervalConfidenceAsFloat(15);
  EXPECT_EQ(confidence_as_float2, 1.0f);

  int random_int_time = randomInt(0, 35990);
  int random_int_seconds = randomInt(0, 3599);
  float time_mark_as_seconds = spatem_ts_access::interpretTimeMarkValueAsSeconds(random_int_time, random_int_seconds, 0);
  EXPECT_EQ(time_mark_as_seconds, (float)random_int_time * 0.1f - random_int_seconds);

  int random_int_time2 = randomInt(0, 35990);
  int random_int_seconds2 = randomInt(0, 3599);
  uint random_uint_nanosecs2 = (uint)randomInt(0, 1e3 - 1) * 1e6;
  double tolerance = 1e-4;

  float time_mark_as_seconds2 = spatem_ts_access::interpretTimeMarkValueAsSeconds(random_int_time2, random_int_seconds2, random_uint_nanosecs2);
  EXPECT_NEAR(time_mark_as_seconds2, (float)random_int_time2 * 0.1f - (random_int_seconds2 + (float)random_uint_nanosecs2 * 1e-9), tolerance);

  spatem_ts_access::time_mark_value_interpretation time_mark_value_type = spatem_ts_access::interpretTimeMarkValueType(36001);
  EXPECT_EQ(time_mark_value_type, spatem_ts_access::time_mark_value_interpretation::undefined);

  spatem_ts_access::time_mark_value_interpretation time_mark_value_type2 = spatem_ts_access::interpretTimeMarkValueType(36000);
  EXPECT_EQ(time_mark_value_type2, spatem_ts_access::time_mark_value_interpretation::over_an_hour);

  spatem_ts_access::time_mark_value_interpretation time_mark_value_type3 = spatem_ts_access::interpretTimeMarkValueType(35991);
  EXPECT_EQ(time_mark_value_type3, spatem_ts_access::time_mark_value_interpretation::leap_second);

  spatem_ts_access::time_mark_value_interpretation time_mark_value_type4 = spatem_ts_access::interpretTimeMarkValueType(500);
  EXPECT_EQ(time_mark_value_type4, spatem_ts_access::time_mark_value_interpretation::normal);

  std::string time_mark_as_string = spatem_ts_access::parseTimeMarkValueToString(36001, 0, 0);
  EXPECT_EQ(time_mark_as_string, std::string("undefined"));

  std::string time_mark_as_string2 = spatem_ts_access::parseTimeMarkValueToString(36000, 0, 0);
  EXPECT_EQ(time_mark_as_string2, std::string(">36000s"));

  std::string time_mark_as_string3 = spatem_ts_access::parseTimeMarkValueToString(35991, 0, 0);
  EXPECT_EQ(time_mark_as_string3, std::string("leap second"));

  std::string time_mark_as_string4 = spatem_ts_access::parseTimeMarkValueToString(55, 0, 0);
  EXPECT_EQ(time_mark_as_string4, std::string("5.5s"));
}