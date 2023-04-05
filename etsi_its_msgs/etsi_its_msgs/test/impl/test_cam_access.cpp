#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>


using namespace etsi_its_cam_msgs;
using namespace etsi_its_cam_msgs::access_functions;


std::uniform_real_distribution<double> uniform_distribution_double(-1, 1);
std::uniform_real_distribution<double> uniform_distribution_double_0_255(0, 255);
std::default_random_engine random_engine;


double randomDouble() {
  return uniform_distribution_double(random_engine);
}

double randomInt() {
  return (int)uniform_distribution_double_0_255(random_engine);
}


TEST(etsi_its_cam_msgs, test_cam_setters) {

  CAM cam;

  int station_id = randomInt();
  int protocol_version = randomInt();
  setItsPduHeader(cam, station_id, protocol_version);
  EXPECT_EQ(ItsPduHeader::MESSAGE_I_D_CAM, cam.header.messageID);
  EXPECT_EQ(protocol_version, cam.header.protocolVersion);
  EXPECT_EQ(station_id, cam.header.stationID.value);

}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
