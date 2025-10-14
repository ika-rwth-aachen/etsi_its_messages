/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include <etsi_its_conversion_srvs/srv/convert_cam_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_cam_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_cpm_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_denm_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_denm_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_mapem_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_mcm_uulm_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_spatem_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_vam_ts_to_udp.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_cam.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_cam_ts.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_cpm_ts.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_denm.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_denm_ts.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_mapem_ts.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_mcm_uulm.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_spatem_ts.hpp>
#include <etsi_its_conversion_srvs/srv/convert_udp_to_vam_ts.hpp>
#include <etsi_its_cam_conversion/convertCAM.h>
#include <etsi_its_cam_ts_conversion/convertCAM.h>
#include <etsi_its_cpm_ts_conversion/convertCollectivePerceptionMessage.h>
#include <etsi_its_denm_conversion/convertDENM.h>
#include <etsi_its_denm_ts_conversion/convertDENM.h>
#include <etsi_its_mapem_ts_conversion/convertMAPEM.h>
#include <etsi_its_mcm_uulm_conversion/convertMCM.h>
#include <etsi_its_spatem_ts_conversion/convertSPATEM.h>
#include <etsi_its_vam_ts_conversion/convertVAM.h>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_cam_ts_msgs/msg/cam.hpp>
#include <etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp>
#include <etsi_its_denm_msgs/msg/denm.hpp>
#include <etsi_its_denm_ts_msgs/msg/denm.hpp>
#include <etsi_its_mapem_ts_msgs/msg/mapem.hpp>
#include <etsi_its_mcm_uulm_msgs/msg/mcm.hpp>
#include <etsi_its_spatem_ts_msgs/msg/spatem.hpp>
#include <etsi_its_vam_ts_msgs/msg/vam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

namespace etsi_its_conversion {

using namespace udp_msgs::msg;
namespace conversion_srvs = etsi_its_conversion_srvs::srv;
namespace cam_msgs = etsi_its_cam_msgs::msg;
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
namespace cpm_ts_msgs = etsi_its_cpm_ts_msgs::msg;
namespace denm_msgs = etsi_its_denm_msgs::msg;
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
namespace mapem_ts_msgs = etsi_its_mapem_ts_msgs::msg;
namespace mcm_uulm_msgs = etsi_its_mcm_uulm_msgs::msg;
namespace spatem_ts_msgs = etsi_its_spatem_ts_msgs::msg;
namespace vam_ts_msgs = etsi_its_vam_ts_msgs::msg;

class Converter : public rclcpp::Node {

  public:
    explicit Converter(const rclcpp::NodeOptions& options);

  protected:
    void loadParameters();

    void setup();

    bool logLevelIsDebug() const;

    template <typename T_struct>
    bool decodeBufferToStruct(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, T_struct* asn1_struct) const;

    template <typename T_ros, typename T_struct>
    T_ros structToRosMessage(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn) const;

    template <typename T_ros, typename T_struct>
    bool decodeBufferToRosMessage(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn, T_ros& msg) const;

    UdpPacket bufferToUdpPacketMessage(const uint8_t* buffer, const int size, const int btp_header_destination_port) const;

    template <typename T_ros, typename T_struct>
    T_struct rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) const;

    template <typename T_struct>
    bool encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t*& buffer, int& size) const;

    template <typename T_ros, typename T_struct>
    bool encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn, const int btp_header_destination_port) const;

    template <typename T_ros, typename T_struct, typename T_request, typename T_response>
    void rosToUdpSrvCallback(const std::shared_ptr<T_request> request,
                            std::shared_ptr<T_response> response,
                            const std::string& type,
                            const asn_TYPE_descriptor_t* asn_type_descriptor,
                            std::function<void(const T_ros&, T_struct&)> conversion_fn) const;

    template <typename T_ros, typename T_struct, typename T_request, typename T_response>
    void udpToRosSrvCallback(const std::shared_ptr<T_request> request, std::shared_ptr<T_response> response, const std::string& type, const asn_TYPE_descriptor_t* asn_type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn) const;

    void udpCallback(const UdpPacket::UniquePtr udp_msg) const;

    template <typename T_ros, typename T_struct>
    void rosCallback(const typename T_ros::UniquePtr msg,
                     const std::string& type, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) const;

  protected:
    static const std::string kInputTopicUdp;
    static const std::string kOutputTopicUdp;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicCam;
    static const std::string kServiceCamToUdp;
    static const std::string kServiceUdpToCam;
    static const std::string kInputTopicCamTs;
    static const std::string kOutputTopicCamTs;
    static const std::string kServiceCamTsToUdp;
    static const std::string kServiceUdpToCamTs;
    static const std::string kInputTopicCpmTs;
    static const std::string kOutputTopicCpmTs;
    static const std::string kServiceCpmTsToUdp;
    static const std::string kServiceUdpToCpmTs;
    static const std::string kInputTopicDenm;
    static const std::string kOutputTopicDenm;
    static const std::string kServiceDenmToUdp;
    static const std::string kServiceUdpToDenm;
    static const std::string kInputTopicDenmTs;
    static const std::string kOutputTopicDenmTs;
    static const std::string kServiceDenmTsToUdp;
    static const std::string kServiceUdpToDenmTs;
    static const std::string kInputTopicMapemTs;
    static const std::string kOutputTopicMapemTs;
    static const std::string kServiceMapemTsToUdp;
    static const std::string kServiceUdpToMapemTs;
    static const std::string kInputTopicMcmUulm;
    static const std::string kOutputTopicMcmUulm;
    static const std::string kServiceMcmUulmToUdp;
    static const std::string kServiceUdpToMcmUulm;
    static const std::string kInputTopicSpatemTs;
    static const std::string kOutputTopicSpatemTs;
    static const std::string kServiceSpatemTsToUdp;
    static const std::string kServiceUdpToSpatemTs;
    static const std::string kInputTopicVamTs;
    static const std::string kOutputTopicVamTs;
    static const std::string kServiceVamTsToUdp;
    static const std::string kServiceUdpToVamTs;

    static const std::string kHasBtpDestinationPortParam;
    static const bool kHasBtpDestinationPortParamDefault;
    static const std::string kBtpDestinationPortOffsetParam;
    static const int kBtpDestinationPortOffsetParamDefault;
    static const std::string kEtsiMessagePayloadOffsetParam;
    static const int kEtsiMessagePayloadOffsetParamDefault;
    static const std::string kRos2UdpEtsiTypesParam;
    static const std::string kUdp2RosEtsiTypesParam;
    static const std::vector<std::string> kEtsiTypesParamSupportedOptions;
    static const std::vector<std::string> kRos2UdpEtsiTypesParamDefault;
    static const std::vector<std::string> kUdp2RosEtsiTypesParamDefault;
    static const std::string kSubscriberQueueSizeParam;
    static const int kSubscriberQueueSizeParamDefault;
    static const std::string kPublisherQueueSizeParam;
    static const int kPublisherQueueSizeParamDefault;
    static const std::string kCheckConstraintsBeforeEncodingParam;
    static const bool kCheckConstraintsBeforeEncodingParamDefault;

    bool has_btp_destination_port_;
    int btp_destination_port_offset_;
    int etsi_message_payload_offset_;
    std::vector<std::string> ros2udp_etsi_types_;
    std::vector<std::string> udp2ros_etsi_types_;
    int subscriber_queue_size_;
    int publisher_queue_size_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<UdpPacket>::SharedPtr subscriber_udp_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;

    rclcpp::Service<conversion_srvs::ConvertCamToUdp>::SharedPtr convert_cam_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertCamTsToUdp>::SharedPtr convert_cam_ts_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertCpmTsToUdp>::SharedPtr convert_cpm_ts_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertDenmToUdp>::SharedPtr convert_denm_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertDenmTsToUdp>::SharedPtr convert_denm_ts_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertMapemTsToUdp>::SharedPtr convert_mapem_ts_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertMcmUulmToUdp>::SharedPtr convert_mcm_uulm_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertSpatemTsToUdp>::SharedPtr convert_spatem_ts_to_udp_service_;
    rclcpp::Service<conversion_srvs::ConvertVamTsToUdp>::SharedPtr convert_vam_ts_to_udp_service_;

    rclcpp::Service<conversion_srvs::ConvertUdpToCam>::SharedPtr convert_udp_to_cam_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToCamTs>::SharedPtr convert_udp_to_cam_ts_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToCpmTs>::SharedPtr convert_udp_to_cpm_ts_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToDenm>::SharedPtr convert_udp_to_denm_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToDenmTs>::SharedPtr convert_udp_to_denm_ts_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToMapemTs>::SharedPtr convert_udp_to_mapem_ts_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToMcmUulm>::SharedPtr convert_udp_to_mcm_uulm_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToSpatemTs>::SharedPtr convert_udp_to_spatem_ts_service_;
    rclcpp::Service<conversion_srvs::ConvertUdpToVamTs>::SharedPtr convert_udp_to_vam_ts_service_;

    mutable rclcpp::Publisher<cam_msgs::CAM>::SharedPtr publisher_cam_;
    mutable rclcpp::Publisher<cam_ts_msgs::CAM>::SharedPtr publisher_cam_ts_;
    mutable rclcpp::Publisher<cpm_ts_msgs::CollectivePerceptionMessage>::SharedPtr publisher_cpm_ts_;
    mutable rclcpp::Publisher<denm_msgs::DENM>::SharedPtr publisher_denm_;
    mutable rclcpp::Publisher<denm_ts_msgs::DENM>::SharedPtr publisher_denm_ts_;
    mutable rclcpp::Publisher<mapem_ts_msgs::MAPEM>::SharedPtr publisher_mapem_ts_;
    mutable rclcpp::Publisher<mcm_uulm_msgs::MCM>::SharedPtr publisher_mcm_uulm_;
    mutable rclcpp::Publisher<spatem_ts_msgs::SPATEM>::SharedPtr publisher_spatem_ts_;
    mutable rclcpp::Publisher<vam_ts_msgs::VAM>::SharedPtr publisher_vam_ts_;
    mutable rclcpp::Publisher<UdpPacket>::SharedPtr publisher_udp_;
};

}
