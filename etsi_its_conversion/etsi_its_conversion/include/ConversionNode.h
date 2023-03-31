#pragma once

#include <memory>
#include <string>

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <etsi_its_cam_conversion/convertCAM.h>
#include <etsi_its_asn1_msgs/ASN1_udp.h>

namespace etsi_its_conversion {

class ConversionNode {

  public:
    ConversionNode();

  private:
    void generateDummyCAM(const ros::TimerEvent& event);
    void cam_asn1_callback(etsi_its_asn1_msgs::ASN1_udp msg);

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Publisher cam_ros_pub_;
    ros::Publisher cam_asn1_pub_;
    ros::Subscriber cam_asn1_sub_;

    ros::Timer timer_;

};


}  // end of namespace etsi_its_conversion
