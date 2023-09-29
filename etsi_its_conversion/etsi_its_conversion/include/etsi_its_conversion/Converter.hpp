#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <etsi_its_cam_conversion/convertCAM.h>
#ifdef ROS1
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <udp_msgs/UdpPacket.h>
#include <etsi_its_cam_msgs/CAM.h>
#else
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#endif


namespace etsi_its_conversion {


#ifdef ROS1
class Converter : public nodelet::Nodelet {
#else
class Converter : public rclcpp::Node {
#endif

  public:

#ifdef ROS1
    virtual void onInit();
#else
    explicit Converter(const rclcpp::NodeOptions& options);
#endif

  protected:

    void loadParameters();

    void setup();

    bool logLevelIsDebug();

#ifdef ROS1
    void udpCallback(const udp_msgs::UdpPacket::ConstPtr udp_msg);
#else
    void udpCallback(const udp_msgs::msg::UdpPacket::UniquePtr udp_msg);
#endif

#ifdef ROS1
    void rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg);
#else
    void rosCallbackCam(const etsi_its_cam_msgs::msg::CAM::UniquePtr msg);
#endif

  protected:

    static const std::string kInputTopicUdp;
    static const std::string kOutputTopicCam;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicUdp;

    static const std::string kEtsiTypeParam;
    static const std::string kEtsiTypeParamDefault;

    std::string etsi_type_;

#ifdef ROS1
    ros::NodeHandle private_node_handle_;
    ros::Subscriber subscriber_udp_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    ros::Publisher publisher_udp_;
#else
    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscriber_udp_;
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr> publishers_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr> subscribers_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher_udp_;
#endif

};


}
