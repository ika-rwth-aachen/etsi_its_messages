#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <etsi_its_cam_conversion/convertCAM.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <udp_msgs/UdpPacket.h>


namespace etsi_its_conversion {


class Converter : public nodelet::Nodelet {

  public:

    virtual void onInit();

  protected:

    void loadParameters();

    void setup();

    void udpCallback(const udp_msgs::UdpPacket::ConstPtr udp_msg);

    void rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg);

  protected:

    static const std::string kInputTopicUdp;
    static const std::string kOutputTopicCam;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicUdp;

    ros::NodeHandle private_node_handle_;

    std::string etsi_type_;

    ros::Subscriber subscriber_udp_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    ros::Publisher publisher_udp_;
};


}
