/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <etsi_its_cam_conversion/convertCAM.h>
#include <etsi_its_denm_conversion/convertDENM.h>
#include <etsi_its_spatem_conversion/convertSPATEM.h>
#include <etsi_its_mapem_conversion/convertMAPEM.h>
#ifdef ROS1
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <udp_msgs/UdpPacket.h>
#include <etsi_its_cam_msgs/CAM.h>
#include <etsi_its_denm_msgs/DENM.h>
#include <etsi_its_spatem_msgs/SPATEM.h>
#include <etsi_its_mapem_msgs/MAPEM.h>
#else
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_denm_msgs/msg/denm.hpp>
#include <etsi_its_spatem_msgs/msg/spatem.hpp>
#include <etsi_its_mapem_msgs/msg/mapem.hpp>
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
    void rosCallbackDenm(const etsi_its_denm_msgs::DENM::ConstPtr msg);
    void rosCallbackSpatem(const etsi_its_spatem_msgs::SPATEM::ConstPtr msg);
    void rosCallbackMapem(const etsi_its_mapem_msgs::MAPEM::ConstPtr msg);
#else
    void rosCallbackCam(const etsi_its_cam_msgs::msg::CAM::UniquePtr msg);
    void rosCallbackDenm(const etsi_its_denm_msgs::msg::DENM::UniquePtr msg);
    void rosCallbackSpatem(const etsi_its_spatem_msgs::msg::SPATEM::UniquePtr msg);
    void rosCallbackMapem(const etsi_its_mapem_msgs::msg::MAPEM::UniquePtr msg);
#endif

  protected:

    static const std::string kInputTopicUdp;
    static const std::string kOutputTopicUdp;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicCam;
    static const std::string kInputTopicDenm;
    static const std::string kOutputTopicDenm;
    static const std::string kInputTopicSpatem;
    static const std::string kOutputTopicSpatem;
    static const std::string kInputTopicMapem;
    static const std::string kOutputTopicMapem;

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
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr> publishers_cam_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr> subscribers_cam_;
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_denm_msgs::msg::DENM>::SharedPtr> publishers_denm_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_denm_msgs::msg::DENM>::SharedPtr> subscribers_denm_;
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_spatem_msgs::msg::SPATEM>::SharedPtr> publishers_spatem_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_spatem_msgs::msg::SPATEM>::SharedPtr> subscribers_spatem_;
    std::unordered_map<std::string, rclcpp::Publisher<etsi_its_mapem_msgs::msg::MAPEM>::SharedPtr> publishers_mapem_;
    std::unordered_map<std::string, rclcpp::Subscription<etsi_its_mapem_msgs::msg::MAPEM>::SharedPtr> subscribers_mapem_;
    rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher_udp_;
#endif

};


}
