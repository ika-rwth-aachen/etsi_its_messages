/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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
#include <etsi_its_cam_ts_conversion/convertCAM.h>
#include <etsi_its_cpm_ts_conversion/convertCollectivePerceptionMessage.h>
#include <etsi_its_denm_conversion/convertDENM.h>
#ifdef ROS1
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <udp_msgs/UdpPacket.h>
#include <etsi_its_cam_msgs/CAM.h>
#include <etsi_its_cam_ts_msgs/CAM.h>
#include <etsi_its_cpm_ts_msgs/CollectivePerceptionMessage.h>
#include <etsi_its_denm_msgs/DENM.h>
#else
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_cam_ts_msgs/msg/cam.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <etsi_its_denm_msgs/msg/denm.hpp>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#endif


namespace etsi_its_conversion {


#ifdef ROS1
using namespace udp_msgs;
namespace cam_msgs = etsi_its_cam_msgs;
namespace cam_ts_msgs = etsi_its_cam_ts_msgs;
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs;
namespace denm_msgs = etsi_its_denm_msgs;
#else
using namespace udp_msgs::msg;
namespace cam_msgs = etsi_its_cam_msgs::msg;
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs::msg;
namespace denm_msgs = etsi_its_denm_msgs::msg;
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

    template <typename T_struct>
    bool decodeBufferToStruct(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, T_struct* asn1_struct);

    template <typename T_ros, typename T_struct>
    T_ros structToRosMessage(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn);

    template <typename T_ros, typename T_struct>
    bool decodeBufferToRosMessage(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn, T_ros& msg);

    UdpPacket bufferToUdpPacketMessage(const uint8_t* buffer, const int size, const int btp_header_destination_port);

    template <typename T_ros, typename T_struct>
    T_struct rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn);

    template <typename T_struct>
    bool encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t*& buffer, int& size);

    template <typename T_ros, typename T_struct>
    bool encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn, const int btp_header_destination_port);

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
    static const std::string kInputTopicCamTs;
    static const std::string kOutputTopicCamTs;
    static const std::string kInputTopicCpmTs;
    static const std::string kOutputTopicCpmTs;
    static const std::string kInputTopicDenm;
    static const std::string kOutputTopicDenm;

    static const std::string kHasBtpDestinationPortParam;
    static const bool kHasBtpDestinationPortParamDefault;
    static const std::string kBtpDestinationPortOffsetParam;
    static const int kBtpDestinationPortOffsetParamDefault;
    static const std::string kEtsiMessagePayloadOffsetParam;
    static const int kEtsiMessagePayloadOffsetParamDefault;
    static const std::string kRos2UdpEtsiTypesParam;
    static const std::string kUdp2RosEtsiTypesParam;
    static const std::vector<std::string> kRos2UdpEtsiTypesParamDefault;
    static const std::vector<std::string> kUdp2RosEtsiTypesParamDefault;
    static const std::string kSubscriberQueueSizeParam;
    static const int kSubscriberQueueSizeParamDefault;
    static const std::string kPublisherQueueSizeParam;
    static const int kPublisherQueueSizeParamDefault;

    bool has_btp_destination_port_;
    int btp_destination_port_offset_;
    int etsi_message_payload_offset_;
    std::vector<std::string> ros2udp_etsi_types_;
    std::vector<std::string> udp2ros_etsi_types_;
    int subscriber_queue_size_;
    int publisher_queue_size_;

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
    rclcpp::Publisher<cam_ts_msgs::CAM>::SharedPtr publisher_cam_ts_;
    rclcpp::Publisher<cpm_ts_msgs::CollectivePerceptionMessage>::SharedPtr publisher_cpm_ts_;
    rclcpp::Publisher<denm_msgs::DENM>::SharedPtr publisher_denm_;
    rclcpp::Publisher<UdpPacket>::SharedPtr publisher_udp_;
#endif

};


}
