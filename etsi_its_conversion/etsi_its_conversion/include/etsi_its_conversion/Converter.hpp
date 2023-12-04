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
using namespace udp_msgs;
namespace cam_msgs = etsi_its_cam_msgs;
namespace denm_msgs = etsi_its_denm_msgs;
namespace spatem_msgs = etsi_its_spatem_msgs;
namespace mapem_msgs = etsi_its_mapem_msgs;
#else
using namespace udp_msgs::msg;
namespace cam_msgs = etsi_its_cam_msgs::msg;
namespace denm_msgs = etsi_its_denm_msgs::msg;
namespace spatem_msgs = etsi_its_spatem_msgs::msg;
namespace mapem_msgs = etsi_its_mapem_msgs::msg;
#endif


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

    UdpPacket bufferToUdpPacketMessage(const uint8_t* buffer, const int size);

    template <typename T_ros, typename T_struct>
    T_struct rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn);

    template <typename T_struct>
    bool encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t* buffer, int& size);

    template <typename T_ros, typename T_struct>
    bool encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn);

#ifdef ROS1
    void udpCallback(const UdpPacket::ConstPtr udp_msg);
#else
    void udpCallback(const UdpPacket::UniquePtr udp_msg);
#endif

    template <typename T_ros, typename T_struct>
#ifdef ROS1
    void rosCallback(const typename T_ros::ConstPtr msg,
#else
    void rosCallback(const typename T_ros::UniquePtr msg,
#endif
                     const std::string& type, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn);

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

    static const std::string kHasBtpHeaderParam;
    static const bool kHasBtpHeaderParamDefault;
    static const std::string kEtsiTypesParam;
    static const std::vector<std::string> kEtsiTypesParamDefault;

    bool has_btp_header_;
    std::vector<std::string> etsi_types_;

#ifdef ROS1
    ros::NodeHandle private_node_handle_;
    std::shared_ptr<ros::Subscriber> subscriber_udp_;
    std::unordered_map<std::string, std::shared_ptr<ros::Publisher>> publishers_;
    std::unordered_map<std::string, std::shared_ptr<ros::Subscriber>> subscribers_;
    std::shared_ptr<ros::Publisher> publisher_udp_;
#else
    rclcpp::Subscription<UdpPacket>::SharedPtr subscriber_udp_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    rclcpp::Publisher<cam_msgs::CAM>::SharedPtr publisher_cam_;
    rclcpp::Publisher<denm_msgs::DENM>::SharedPtr publisher_denm_;
    rclcpp::Publisher<spatem_msgs::SPATEM>::SharedPtr publisher_spatem_;
    rclcpp::Publisher<mapem_msgs::MAPEM>::SharedPtr publisher_mapem_;
    rclcpp::Publisher<UdpPacket>::SharedPtr publisher_udp_;
#endif

};


}
