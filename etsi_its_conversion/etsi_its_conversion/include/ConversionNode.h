#pragma once

#include <memory>
#include <string>

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <etsi_its_cam_conversion/convertCAM.h>

namespace etsi_its_conversion {

class ConversionNode {

  public:
    ConversionNode();

  private:
    void generateDummyCAM(const ros::TimerEvent& event);
    void CAMCallback(const etsi_its_cam_msgs::CAM& msg);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    ros::Timer timer_;

};


}  // end of namespace etsi_its_conversion
