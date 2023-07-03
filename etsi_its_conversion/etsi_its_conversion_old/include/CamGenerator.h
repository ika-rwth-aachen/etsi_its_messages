#pragma once

#include <memory>
#include <string>

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <bitstring_msgs/UInt8ArrayStamped.h>
#include <etsi_its_cam_coding/CAM.h>

namespace etsi_its_conversion {

class CamGeneratorNode {

  public:
    CamGeneratorNode();

  private:
    void generateDummyCAM(const ros::TimerEvent& event);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher asn1_pub_;

    ros::Timer timer_;

};


}  // end of namespace etsi_its_conversion