#pragma once

#include <memory>
#include <string>

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <convertCAM.h>

namespace sample_package {

class SampleNode {

  public:
    SampleNode();

  private:
    void timerCallback(const ros::TimerEvent& event);
    void messageCallback(const etsi_its_cam_msgs::CAM& msg);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    ros::Timer timer_;

};


}  // end of namespace sample_package
