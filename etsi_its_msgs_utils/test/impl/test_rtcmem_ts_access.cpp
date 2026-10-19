#include <gtest/gtest.h>
namespace rtcmem_ts_access = etsi_its_rtcmem_ts_msgs::access;

TEST(etsi_its_rtcmem_ts_msgs, test_set_get_rtcmem) {
  rtcmem_ts_msgs::RTCMEM rtcmem;

  uint32_t station_id = randomInt(rtcmem_ts_msgs::StationID::MIN, rtcmem_ts_msgs::StationID::MAX);
  uint8_t protocol_version = randomInt(rtcmem_ts_msgs::ItsPduHeader::PROTOCOL_VERSION_MIN,
                                       rtcmem_ts_msgs::ItsPduHeader::PROTOCOL_VERSION_MAX);
  rtcmem_ts_access::setProtocolVersion(rtcmem, protocol_version);
  rtcmem_ts_access::setMessageIDToRTCMEM(rtcmem);
  rtcmem_ts_access::setStationID(rtcmem, station_id);
  EXPECT_EQ(rtcmem_ts_msgs::ItsPduHeader::MESSAGE_ID_RTCMEM, rtcmem_ts_access::getMessageID(rtcmem));
  EXPECT_EQ(protocol_version, rtcmem_ts_access::getProtocolVersion(rtcmem));
  EXPECT_EQ(station_id, rtcmem_ts_access::getStationIDValue(rtcmem));

  uint8_t message_count = randomInt(rtcmem_ts_msgs::MsgCount::MIN, rtcmem_ts_msgs::MsgCount::MAX);
  rtcmem_ts_access::setMsgCount(rtcmem, message_count);
  EXPECT_EQ(message_count, rtcmem_ts_access::getMsgCount(rtcmem));

  uint8_t revision = randomInt(rtcmem_ts_msgs::RTCMRevision::UNKNOWN, rtcmem_ts_msgs::RTCMRevision::RESERVED);
  rtcmem_ts_access::setRTCMRevision(rtcmem, revision);
  EXPECT_EQ(revision, rtcmem_ts_access::getRTCMRevisionValue(rtcmem));

  uint32_t minute_of_year = randomInt(rtcmem_ts_msgs::MinuteOfTheYear::MIN,
                                      rtcmem_ts_msgs::MinuteOfTheYear::MAX);
  rtcmem_ts_access::setMinuteOfTheYear(rtcmem, minute_of_year);
  EXPECT_TRUE(rtcmem_ts_access::hasMinuteOfTheYear(rtcmem));
  EXPECT_EQ(minute_of_year, rtcmem_ts_access::getMinuteOfTheYearValue(rtcmem));

  struct tm timeinfo{};
  timeinfo.tm_sec = 0;
  timeinfo.tm_min = 0;
  timeinfo.tm_hour = 0;
  timeinfo.tm_mday = 1;
  timeinfo.tm_mon = 0;
  timeinfo.tm_year = 126; // year 2026
  uint64_t unix_stamp = timegm(&timeinfo);
  EXPECT_EQ((unix_stamp + 60 * minute_of_year) * 1e9,
            rtcm_ts_access::getUnixNanoseconds(rtcmem, unix_stamp * 1e9));

  rtcmem_ts_msgs::RTCMmessage rtcm_message;
  std::vector<uint8_t> rtcm_message_data{0xD3, 0x00, 0x13};
  rtcm_ts_access::setRTCMmessageValue(rtcm_message, rtcm_message_data);
  rtcm_ts_access::setRTCMmessageList(rtcmem, {rtcm_message});
  EXPECT_EQ(1u, rtcm_ts_access::getRTCMmessageCount(rtcmem));
  EXPECT_EQ(rtcm_message_data, rtcm_ts_access::getRTCMmessageValue(rtcm_ts_access::getRTCMmessage(rtcmem, 0)));

  rtcm_ts_access::clearMinuteOfTheYear(rtcmem);
  EXPECT_FALSE(rtcm_ts_access::hasMinuteOfTheYear(rtcmem));
}
