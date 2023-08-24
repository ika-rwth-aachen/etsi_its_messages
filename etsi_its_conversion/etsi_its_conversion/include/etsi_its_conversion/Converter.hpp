#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <etsi_its_cam_conversion/convertCAM.h>
#ifndef ROS1
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#else
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <udp_msgs/UdpPacket.h>
#include <etsi_its_cam_msgs/CAM.h>
#endif


namespace etsi_its_conversion {


#ifndef ROS1
class Converter : public rclcpp::Node {
#else
class Converter : public nodelet::Nodelet {
#endif

  public:

#ifndef ROS1
    Converter();
#else
    virtual void onInit();
#endif

  protected:

    void loadParameters();

    void setup();

#ifndef ROS1
    void udpCallback(const udp_msgs::msg::UdpPacket::SharedPtr udp_msg);
#else
    void udpCallback(const udp_msgs::UdpPacket::ConstPtr udp_msg);
#endif

#ifndef ROS1
    void rosCallbackCam(const etsi_its_cam_msgs::msg::CAM::SharedPtr msg);
#else
    void rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg);
#endif

  protected:

    static const std::string kInputTopicUdp;
    static const std::string kOutputTopicCam;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicUdp;

    static const std::string kEtsiTypeParam;
    static const std::string kEtsiTypeParamDefault;

    std::string etsi_type_;

#ifndef ROS1
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscriber_udp_;
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr> publishers_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr> subscribers_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher_udp_;
#else
    ros::NodeHandle private_node_handle_;
    ros::Subscriber subscriber_udp_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    ros::Publisher publisher_udp_;
#endif

};


}
