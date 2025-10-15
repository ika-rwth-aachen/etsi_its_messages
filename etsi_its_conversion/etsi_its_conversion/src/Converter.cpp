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

#include <algorithm>
#include <arpa/inet.h>
#include <sstream>

#include <rcutils/logging.h>

#include "etsi_its_conversion/Converter.hpp"

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(etsi_its_conversion::Converter)

namespace etsi_its_conversion {

#ifdef ROS_DISTRO_HUMBLE
#define SERVICE_QOS rmw_qos_profile_services_default
#else
#define SERVICE_QOS rclcpp::ServicesQoS()
#endif

const int kBtpHeaderDestinationPortCam{2001};
const int kBtpHeaderDestinationPortDenm{2002};
const int kBtpHeaderDestinationPortMapem{2003};
const int kBtpHeaderDestinationPortSpatem{2004};
const int kBtpHeaderDestinationPortIvi{2006};
const int kBtpHeaderDestinationPortCpmTs{2009};
const int kBtpHeaderDestinationPortVamTs{2018};
const int kBtpHeaderDestinationPortMcmUulm{2020};

const std::string Converter::kInputTopicUdp{"~/udp/in"};
const std::string Converter::kOutputTopicUdp{"~/udp/out"};
const std::string Converter::kInputTopicCam{"~/cam/in"};
const std::string Converter::kOutputTopicCam{"~/cam/out"};
const std::string Converter::kServiceCamToUdp{"~/cam/udp"};
const std::string Converter::kServiceUdpToCam{"~/udp/cam"};
const std::string Converter::kInputTopicCamTs{"~/cam_ts/in"};
const std::string Converter::kOutputTopicCamTs{"~/cam_ts/out"};
const std::string Converter::kServiceCamTsToUdp{"~/cam_ts/udp"};
const std::string Converter::kServiceUdpToCamTs{"~/udp/cam_ts"};
const std::string Converter::kInputTopicCpmTs{"~/cpm_ts/in"};
const std::string Converter::kOutputTopicCpmTs{"~/cpm_ts/out"};
const std::string Converter::kServiceCpmTsToUdp{"~/cpm_ts/udp"};
const std::string Converter::kServiceUdpToCpmTs{"~/udp/cpm_ts"};
const std::string Converter::kInputTopicDenm{"~/denm/in"};
const std::string Converter::kOutputTopicDenm{"~/denm/out"};
const std::string Converter::kServiceDenmToUdp{"~/denm/udp"};
const std::string Converter::kServiceUdpToDenm{"~/udp/denm"};
const std::string Converter::kInputTopicDenmTs{"~/denm_ts/in"};
const std::string Converter::kOutputTopicDenmTs{"~/denm_ts/out"};
const std::string Converter::kServiceDenmTsToUdp{"~/denm_ts/udp"};
const std::string Converter::kServiceUdpToDenmTs{"~/udp/denm_ts"};
const std::string Converter::kInputTopicMapemTs{"~/mapem_ts/in"};
const std::string Converter::kOutputTopicMapemTs{"~/mapem_ts/out"};
const std::string Converter::kServiceMapemTsToUdp{"~/mapem_ts/udp"};
const std::string Converter::kServiceUdpToMapemTs{"~/udp/mapem_ts"};
const std::string Converter::kInputTopicMcmUulm{"~/mcm_uulm/in"};
const std::string Converter::kOutputTopicMcmUulm{"~/mcm_uulm/out"};
const std::string Converter::kServiceMcmUulmToUdp{"~/mcm_uulm/udp"};
const std::string Converter::kServiceUdpToMcmUulm{"~/udp/mcm_uulm"};
const std::string Converter::kInputTopicSpatemTs{"~/spatem_ts/in"};
const std::string Converter::kOutputTopicSpatemTs{"~/spatem_ts/out"};
const std::string Converter::kServiceSpatemTsToUdp{"~/spatem_ts/udp"};
const std::string Converter::kServiceUdpToSpatemTs{"~/udp/spatem_ts"};
const std::string Converter::kInputTopicVamTs{"~/vam_ts/in"};
const std::string Converter::kOutputTopicVamTs{"~/vam_ts/out"};
const std::string Converter::kServiceVamTsToUdp{"~/vam_ts/udp"};
const std::string Converter::kServiceUdpToVamTs{"~/udp/vam_ts"};

const std::string Converter::kHasBtpDestinationPortParam{"has_btp_destination_port"};
const bool Converter::kHasBtpDestinationPortParamDefault{true};
const std::string Converter::kBtpDestinationPortOffsetParam{"btp_destination_port_offset"};
const int Converter::kBtpDestinationPortOffsetParamDefault{0};
const std::string Converter::kEtsiMessagePayloadOffsetParam{"etsi_message_payload_offset"};
const int Converter::kEtsiMessagePayloadOffsetParamDefault{4};
const std::string Converter::kRos2UdpEtsiTypesParam{"ros2udp_etsi_types"};
const std::string Converter::kUdp2RosEtsiTypesParam{"udp2ros_etsi_types"};
const std::vector<std::string> Converter::kEtsiTypesParamSupportedOptions{"cam", "cam_ts", "cpm_ts", "denm", "denm_ts", "mapem_ts", "mcm_uulm", "spatem_ts", "vam_ts"};
const std::vector<std::string> Converter::kRos2UdpEtsiTypesParamDefault = Converter::kEtsiTypesParamSupportedOptions;
const std::vector<std::string> Converter::kUdp2RosEtsiTypesParamDefault{"cam", "cpm_ts", "denm", "mapem_ts", "mcm_uulm", "spatem_ts", "vam_ts"};
const std::string Converter::kSubscriberQueueSizeParam{"subscriber_queue_size"};
const int Converter::kSubscriberQueueSizeParamDefault{10};
const std::string Converter::kPublisherQueueSizeParam{"publisher_queue_size"};
const int Converter::kPublisherQueueSizeParamDefault{10};
const bool Converter::kCheckConstraintsBeforeEncodingParamDefault{false};


bool Converter::logLevelIsDebug() const {

  auto logger_level = rcutils_logging_get_logger_effective_level(this->get_logger().get_name());
  return (logger_level == RCUTILS_LOG_SEVERITY_DEBUG);
}


Converter::Converter(const rclcpp::NodeOptions& options) : Node("converter", options) {

  this->loadParameters();
  this->setup();
}


void Converter::loadParameters() {

  rcl_interfaces::msg::ParameterDescriptor param_desc;

  // load has_btp_destination_port
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "whether incoming/outgoing UDP messages include a 4-byte BTP header";
  this->declare_parameter(kHasBtpDestinationPortParam, kHasBtpDestinationPortParamDefault, param_desc);
  if (!this->get_parameter(kHasBtpDestinationPortParam, has_btp_destination_port_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%s'", kHasBtpDestinationPortParam.c_str(), kHasBtpDestinationPortParamDefault ? "true" : "false");
  }

  // load btp_destination_port_offset
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kBtpDestinationPortOffsetParam, kBtpDestinationPortOffsetParamDefault, param_desc);
  if (!this->get_parameter(kBtpDestinationPortOffsetParam, btp_destination_port_offset_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%d'", kBtpDestinationPortOffsetParam.c_str(), kBtpDestinationPortOffsetParamDefault);
  }

  // load etsi_message_payload_offset
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kEtsiMessagePayloadOffsetParam, kEtsiMessagePayloadOffsetParamDefault, param_desc);
  if (!this->get_parameter(kEtsiMessagePayloadOffsetParam, etsi_message_payload_offset_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%d'", kEtsiMessagePayloadOffsetParam.c_str(), kEtsiMessagePayloadOffsetParamDefault);
  }

  // load ros2udp_etsi_types_
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss_ros2udp;
  ss_ros2udp << "list of ETSI types to convert from ROS to UDP, one of ";
  for (const auto& e : kRos2UdpEtsiTypesParamDefault) ss_ros2udp << e << ", ";
  param_desc.description = ss_ros2udp.str();
  this->declare_parameter(kRos2UdpEtsiTypesParam, kRos2UdpEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kRos2UdpEtsiTypesParam, ros2udp_etsi_types_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to all", kRos2UdpEtsiTypesParam.c_str());
  }

  // check ros2udp_etsi_types
  for (auto it = ros2udp_etsi_types_.begin(); it != ros2udp_etsi_types_.end(); ) {
    if (std::find(kEtsiTypesParamSupportedOptions.begin(), kEtsiTypesParamSupportedOptions.end(), *it) == kEtsiTypesParamSupportedOptions.end()) {
      RCLCPP_WARN(this->get_logger(), "Invalid value '%s' for parameter '%s', removing", it->c_str(), kRos2UdpEtsiTypesParam.c_str());
      it = ros2udp_etsi_types_.erase(it);
    } else {
      ++it;
    }
  }

  // load udp2ros_etsi_types_
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss_udp2ros;
  ss_udp2ros << "list of ETSI types to convert from UDP to ROS, one of ";
  for (const auto& e : kUdp2RosEtsiTypesParamDefault) ss_udp2ros << e << ", ";
  param_desc.description = ss_udp2ros.str();
  this->declare_parameter(kUdp2RosEtsiTypesParam, kUdp2RosEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kUdp2RosEtsiTypesParam, udp2ros_etsi_types_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to all", kUdp2RosEtsiTypesParam.c_str());
  }

  // check udp2ros_etsi_types
  for (auto it = udp2ros_etsi_types_.begin(); it != udp2ros_etsi_types_.end(); ) {
    if (std::find(kEtsiTypesParamSupportedOptions.begin(), kEtsiTypesParamSupportedOptions.end(), *it) == kEtsiTypesParamSupportedOptions.end()) {
      RCLCPP_WARN(this->get_logger(), "Invalid value '%s' for parameter '%s', removing", it->c_str(), kRos2UdpEtsiTypesParam.c_str());
      it = udp2ros_etsi_types_.erase(it);
    } else {
      ++it;
    }
  }
  if (!has_btp_destination_port_ && udp2ros_etsi_types_.size() > 1) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is set to 'false', but multiple 'udp2ros_etsi_types' are configured, dropping all but the first", kHasBtpDestinationPortParam.c_str());
    udp2ros_etsi_types_.resize(1);
  }

  // load subscriber_queue_size
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "queue size for incoming ROS messages";
  this->declare_parameter(kSubscriberQueueSizeParam, kSubscriberQueueSizeParamDefault, param_desc);
  if (!this->get_parameter(kSubscriberQueueSizeParam, subscriber_queue_size_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%d'", kSubscriberQueueSizeParam.c_str(), kSubscriberQueueSizeParamDefault);
  }

  // load publisher_queue_size
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "queue size for outgoing ROS messages";
  this->declare_parameter(kPublisherQueueSizeParam, kPublisherQueueSizeParamDefault, param_desc);
  if (!this->get_parameter(kPublisherQueueSizeParam, publisher_queue_size_)) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%d'", kPublisherQueueSizeParam.c_str(), kPublisherQueueSizeParamDefault);
  }
}


void Converter::setup() {

  // create reentrant callback group for multi-threaded parallel subscription callback execution
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subscriber_options;
  subscriber_options.callback_group = callback_group_;

  // create subscribers and publishers
  if (!ros2udp_etsi_types_.empty()) {
    publisher_udp_ = this->create_publisher<UdpPacket>(kOutputTopicUdp, publisher_queue_size_);
  }
  if (!udp2ros_etsi_types_.empty()) {
    subscriber_udp_ = this->create_subscription<UdpPacket>(kInputTopicUdp, subscriber_queue_size_, std::bind(&Converter::udpCallback, this, std::placeholders::_1), subscriber_options);
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam") != udp2ros_etsi_types_.end()) {
    convert_udp_to_cam_service_ = this->create_service<conversion_srvs::ConvertUdpToCam>(
      kServiceUdpToCam,
      std::bind(
        &Converter::udpToRosSrvCallback<cam_msgs::CAM, cam_CAM_t, conversion_srvs::ConvertUdpToCam>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cam",
        &asn_DEF_cam_CAM,
        std::function<void(const cam_CAM_t &, cam_msgs::CAM &)>(etsi_its_cam_conversion::toRos_CAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_cam_ = this->create_publisher<cam_msgs::CAM>(kOutputTopicCam, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cam_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CAM to native ROS messages via service '%s'", convert_udp_to_cam_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam") != ros2udp_etsi_types_.end()) {
    convert_cam_to_udp_service_ = this->create_service<conversion_srvs::ConvertCamToUdp>(
      kServiceCamToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<cam_msgs::CAM, cam_CAM_t, conversion_srvs::ConvertCamToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cam",
        &asn_DEF_cam_CAM,
        std::function<void(const cam_msgs::CAM &, cam_CAM_t &)>(etsi_its_cam_conversion::toStruct_CAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const cam_msgs::CAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cam_msgs::CAM, cam_CAM_t>, this, std::placeholders::_1, "cam", &asn_DEF_cam_CAM, std::function<void(const cam_msgs::CAM&, cam_CAM_t&)>(etsi_its_cam_conversion::toStruct_CAM));
    subscribers_["cam"] = this->create_subscription<cam_msgs::CAM>(kInputTopicCam, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CAMs to UDP messages via service '%s'", convert_cam_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_cam_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToCamTs>(
      kServiceUdpToCamTs,
      std::bind(
        &Converter::udpToRosSrvCallback<cam_ts_msgs::CAM, cam_ts_CAM_t, conversion_srvs::ConvertUdpToCamTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cam_ts",
        &asn_DEF_cam_ts_CAM,
        std::function<void(const cam_ts_CAM_t &, cam_ts_msgs::CAM &)>(etsi_its_cam_ts_conversion::toRos_CAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_cam_ts_ = this->create_publisher<cam_ts_msgs::CAM>(kOutputTopicCamTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CAM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cam_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CAM (TS) to native ROS messages via service '%s'", convert_udp_to_cam_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam_ts") != ros2udp_etsi_types_.end()) {
    convert_cam_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertCamTsToUdp>(
      kServiceCamTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<cam_ts_msgs::CAM, cam_ts_CAM_t, conversion_srvs::ConvertCamTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cam_ts",
        &asn_DEF_cam_ts_CAM,
        std::function<void(const cam_ts_msgs::CAM &, cam_ts_CAM_t &)>(etsi_its_cam_ts_conversion::toStruct_CAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const cam_ts_msgs::CAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cam_ts_msgs::CAM, cam_ts_CAM_t>, this, std::placeholders::_1, "cam_ts", &asn_DEF_cam_ts_CAM, std::function<void(const cam_ts_msgs::CAM&, cam_ts_CAM_t&)>(etsi_its_cam_ts_conversion::toStruct_CAM));
    subscribers_["cam_ts"] = this->create_subscription<cam_ts_msgs::CAM>(kInputTopicCamTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CAM (TS) on '%s' to UDP messages on '%s'", subscribers_["cam_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CAM (TS) to UDP messages via service '%s'", convert_cam_ts_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cpm_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_cpm_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToCpmTs>(
      kServiceUdpToCpmTs,
      std::bind(
        &Converter::udpToRosSrvCallback<cpm_ts_msgs::CollectivePerceptionMessage, cpm_ts_CollectivePerceptionMessage_t, conversion_srvs::ConvertUdpToCpmTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cpm_ts",
        &asn_DEF_cpm_ts_CollectivePerceptionMessage,
        std::function<void(const cpm_ts_CollectivePerceptionMessage_t &, cpm_ts_msgs::CollectivePerceptionMessage &)>(etsi_its_cpm_ts_conversion::toRos_CollectivePerceptionMessage)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_cpm_ts_ = this->create_publisher<cpm_ts_msgs::CollectivePerceptionMessage>(kOutputTopicCpmTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CPM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cpm_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CPM (TS) to native ROS messages via service '%s'", convert_udp_to_cpm_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cpm_ts") != ros2udp_etsi_types_.end()) {
    convert_cpm_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertCpmTsToUdp>(
      kServiceCpmTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<cpm_ts_msgs::CollectivePerceptionMessage, cpm_ts_CollectivePerceptionMessage_t, conversion_srvs::ConvertCpmTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "cpm_ts",
        &asn_DEF_cpm_ts_CollectivePerceptionMessage,
        std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage &, cpm_ts_CollectivePerceptionMessage_t &)>(etsi_its_cpm_ts_conversion::toStruct_CollectivePerceptionMessage)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cpm_ts_msgs::CollectivePerceptionMessage, cpm_ts_CollectivePerceptionMessage_t>, this, std::placeholders::_1, "cpm_ts", &asn_DEF_cpm_ts_CollectivePerceptionMessage, std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage&, cpm_ts_CollectivePerceptionMessage_t&)>(etsi_its_cpm_ts_conversion::toStruct_CollectivePerceptionMessage));
    subscribers_["cpm_ts"] = this->create_subscription<cpm_ts_msgs::CollectivePerceptionMessage>(kInputTopicCpmTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CPMs (TS) on '%s' to UDP messages on '%s'", subscribers_["cpm_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CPMs (TS) to UDP messages via service '%s'", convert_cpm_ts_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm") != udp2ros_etsi_types_.end()) {
    convert_udp_to_denm_service_ = this->create_service<conversion_srvs::ConvertUdpToDenm>(
      kServiceUdpToDenm,
      std::bind(
        &Converter::udpToRosSrvCallback<denm_msgs::DENM, denm_DENM_t, conversion_srvs::ConvertUdpToDenm>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "denm",
        &asn_DEF_denm_DENM,
        std::function<void(const denm_DENM_t &, denm_msgs::DENM &)>(etsi_its_denm_conversion::toRos_DENM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_denm_ = this->create_publisher<denm_msgs::DENM>(kOutputTopicDenm, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_denm_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type DENM to native ROS messages via service '%s'", convert_udp_to_denm_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "denm") != ros2udp_etsi_types_.end()) {
    convert_denm_to_udp_service_ = this->create_service<conversion_srvs::ConvertDenmToUdp>(
      kServiceDenmToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<denm_msgs::DENM, denm_DENM_t, conversion_srvs::ConvertDenmToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "denm",
        &asn_DEF_denm_DENM,
        std::function<void(const denm_msgs::DENM &, denm_DENM_t &)>(etsi_its_denm_conversion::toStruct_DENM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const denm_msgs::DENM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<denm_msgs::DENM, denm_DENM_t>, this, std::placeholders::_1, "denm", &asn_DEF_denm_DENM, std::function<void(const denm_msgs::DENM&, denm_DENM_t&)>(etsi_its_denm_conversion::toStruct_DENM));
    subscribers_["denm"] = this->create_subscription<denm_msgs::DENM>(kInputTopicDenm, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs to UDP messages via service '%s'", convert_denm_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_denm_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToDenmTs>(
      kServiceUdpToDenmTs,
      std::bind(
        &Converter::udpToRosSrvCallback<denm_ts_msgs::DENM, denm_ts_DENM_t, conversion_srvs::ConvertUdpToDenmTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "denm_ts",
        &asn_DEF_denm_ts_DENM,
        std::function<void(const denm_ts_DENM_t &, denm_ts_msgs::DENM &)>(etsi_its_denm_ts_conversion::toRos_DENM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_denm_ts_ = this->create_publisher<denm_ts_msgs::DENM>(kOutputTopicDenmTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type DENM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_denm_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type DENM (TS) to native ROS messages via service '%s'", convert_udp_to_denm_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "denm_ts") != ros2udp_etsi_types_.end()) {
    convert_denm_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertDenmTsToUdp>(
      kServiceDenmTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<denm_ts_msgs::DENM, denm_ts_DENM_t, conversion_srvs::ConvertDenmTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "denm_ts",
        &asn_DEF_denm_ts_DENM,
        std::function<void(const denm_ts_msgs::DENM &, denm_ts_DENM_t &)>(etsi_its_denm_ts_conversion::toStruct_DENM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const denm_ts_msgs::DENM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<denm_ts_msgs::DENM, denm_ts_DENM_t>, this, std::placeholders::_1, "denm_ts", &asn_DEF_denm_ts_DENM, std::function<void(const denm_ts_msgs::DENM&, denm_ts_DENM_t&)>(etsi_its_denm_ts_conversion::toStruct_DENM));
    subscribers_["denm_ts"] = this->create_subscription<denm_ts_msgs::DENM>(kInputTopicDenmTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs (TS) on '%s' to UDP messages on '%s'", subscribers_["denm_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs (TS) to UDP messages via service '%s'", convert_denm_ts_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "mapem_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_mapem_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToMapemTs>(
      kServiceUdpToMapemTs,
      std::bind(
        &Converter::udpToRosSrvCallback<mapem_ts_msgs::MAPEM, mapem_ts_MAPEM_t, conversion_srvs::ConvertUdpToMapemTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "mapem_ts",
        &asn_DEF_mapem_ts_MAPEM,
        std::function<void(const mapem_ts_MAPEM_t &, mapem_ts_msgs::MAPEM &)>(etsi_its_mapem_ts_conversion::toRos_MAPEM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_mapem_ts_ = this->create_publisher<mapem_ts_msgs::MAPEM>(kOutputTopicMapemTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type MAPEM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_mapem_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type MAPEM (TS) to native ROS messages via service '%s'", convert_udp_to_mapem_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "mapem_ts") != ros2udp_etsi_types_.end()) {
    convert_mapem_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertMapemTsToUdp>(
      kServiceMapemTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<mapem_ts_msgs::MAPEM, mapem_ts_MAPEM_t, conversion_srvs::ConvertMapemTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "mapem_ts",
        &asn_DEF_mapem_ts_MAPEM,
        std::function<void(const mapem_ts_msgs::MAPEM &, mapem_ts_MAPEM_t &)>(etsi_its_mapem_ts_conversion::toStruct_MAPEM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const mapem_ts_msgs::MAPEM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<mapem_ts_msgs::MAPEM, mapem_ts_MAPEM_t>, this, std::placeholders::_1, "mapem_ts", &asn_DEF_mapem_ts_MAPEM, std::function<void(const mapem_ts_msgs::MAPEM&, mapem_ts_MAPEM_t&)>(etsi_its_mapem_ts_conversion::toStruct_MAPEM));
    subscribers_["mapem_ts"] = this->create_subscription<mapem_ts_msgs::MAPEM>(kInputTopicMapemTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS MAPEMs (TS) on '%s' to UDP messages on '%s'", subscribers_["mapem_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS MAPEMs (TS) to UDP messages via service '%s'", convert_mapem_ts_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "mcm_uulm") != udp2ros_etsi_types_.end()) {
    convert_udp_to_mcm_uulm_service_ = this->create_service<conversion_srvs::ConvertUdpToMcmUulm>(
      kServiceUdpToMcmUulm,
      std::bind(
        &Converter::udpToRosSrvCallback<mcm_uulm_msgs::MCM, mcm_uulm_MCM_t, conversion_srvs::ConvertUdpToMcmUulm>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "mcm_uulm",
        &asn_DEF_mcm_uulm_MCM,
        std::function<void(const mcm_uulm_MCM_t &, mcm_uulm_msgs::MCM &)>(etsi_its_mcm_uulm_conversion::toRos_MCM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_mcm_uulm_ = this->create_publisher<mcm_uulm_msgs::MCM>(kOutputTopicMcmUulm, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type MCM (UULM) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_mcm_uulm_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type MCM (UULM) to native ROS messages via service '%s'", convert_udp_to_mcm_uulm_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "mcm_uulm") != ros2udp_etsi_types_.end()) {
    convert_mcm_uulm_to_udp_service_ = this->create_service<conversion_srvs::ConvertMcmUulmToUdp>(
      kServiceMcmUulmToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<mcm_uulm_msgs::MCM, mcm_uulm_MCM_t, conversion_srvs::ConvertMcmUulmToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "mcm_uulm",
        &asn_DEF_mcm_uulm_MCM,
        std::function<void(const mcm_uulm_msgs::MCM &, mcm_uulm_MCM_t &)>(etsi_its_mcm_uulm_conversion::toStruct_MCM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const mcm_uulm_msgs::MCM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<mcm_uulm_msgs::MCM, mcm_uulm_MCM_t>, this, std::placeholders::_1, "mcm_uulm", &asn_DEF_mcm_uulm_MCM, std::function<void(const mcm_uulm_msgs::MCM&, mcm_uulm_MCM_t&)>(etsi_its_mcm_uulm_conversion::toStruct_MCM));
    subscribers_["mcm_uulm"] = this->create_subscription<mcm_uulm_msgs::MCM>(kInputTopicMcmUulm, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS MCM (UULM) on '%s' to UDP messages on '%s'", subscribers_["mcm_uulm"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS MCM (UULM) to UDP messages via service '%s'", convert_mcm_uulm_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "spatem_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_spatem_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToSpatemTs>(
      kServiceUdpToSpatemTs,
      std::bind(
        &Converter::udpToRosSrvCallback<spatem_ts_msgs::SPATEM, spatem_ts_SPATEM_t, conversion_srvs::ConvertUdpToSpatemTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "spatem_ts",
        &asn_DEF_spatem_ts_SPATEM,
        std::function<void(const spatem_ts_SPATEM_t &, spatem_ts_msgs::SPATEM &)>(etsi_its_spatem_ts_conversion::toRos_SPATEM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_spatem_ts_ = this->create_publisher<spatem_ts_msgs::SPATEM>(kOutputTopicSpatemTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type SPATEM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_spatem_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type SPATEM (TS) to native ROS messages via service '%s'", convert_udp_to_spatem_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "spatem_ts") != ros2udp_etsi_types_.end()) {
    convert_spatem_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertSpatemTsToUdp>(
      kServiceSpatemTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<spatem_ts_msgs::SPATEM, spatem_ts_SPATEM_t, conversion_srvs::ConvertSpatemTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "spatem_ts",
        &asn_DEF_spatem_ts_SPATEM,
        std::function<void(const spatem_ts_msgs::SPATEM &, spatem_ts_SPATEM_t &)>(etsi_its_spatem_ts_conversion::toStruct_SPATEM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const spatem_ts_msgs::SPATEM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<spatem_ts_msgs::SPATEM, spatem_ts_SPATEM_t>, this, std::placeholders::_1, "spatem_ts", &asn_DEF_spatem_ts_SPATEM, std::function<void(const spatem_ts_msgs::SPATEM&, spatem_ts_SPATEM_t&)>(etsi_its_spatem_ts_conversion::toStruct_SPATEM));
    subscribers_["spatem_ts"] = this->create_subscription<spatem_ts_msgs::SPATEM>(kInputTopicSpatemTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS SPATEMs (TS) on '%s' to UDP messages on '%s'", subscribers_["spatem_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS SPATEMs (TS) to UDP messages via service '%s'", convert_spatem_ts_to_udp_service_->get_service_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "vam_ts") != udp2ros_etsi_types_.end()) {
    convert_udp_to_vam_ts_service_ = this->create_service<conversion_srvs::ConvertUdpToVamTs>(
      kServiceUdpToVamTs,
      std::bind(
        &Converter::udpToRosSrvCallback<vam_ts_msgs::VAM, vam_ts_VAM_t, conversion_srvs::ConvertUdpToVamTs>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "vam_ts",
        &asn_DEF_vam_ts_VAM,
        std::function<void(const vam_ts_VAM_t &, vam_ts_msgs::VAM &)>(etsi_its_vam_ts_conversion::toRos_VAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    publisher_vam_ts_ = this->create_publisher<vam_ts_msgs::VAM>(kOutputTopicVamTs, publisher_queue_size_);
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type VAM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_vam_ts_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type VAM (TS) to native ROS messages via service '%s'", convert_udp_to_vam_ts_service_->get_service_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "vam_ts") != ros2udp_etsi_types_.end()) {
    convert_vam_ts_to_udp_service_ = this->create_service<conversion_srvs::ConvertVamTsToUdp>(
      kServiceVamTsToUdp,
      std::bind(
        &Converter::rosToUdpSrvCallback<vam_ts_msgs::VAM, vam_ts_VAM_t, conversion_srvs::ConvertVamTsToUdp>,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        "vam_ts",
        &asn_DEF_vam_ts_VAM,
        std::function<void(const vam_ts_msgs::VAM &, vam_ts_VAM_t &)>(etsi_its_vam_ts_conversion::toStruct_VAM)
      ),
      SERVICE_QOS,
      callback_group_
    );
    std::function<void(const vam_ts_msgs::VAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<vam_ts_msgs::VAM, vam_ts_VAM_t>, this, std::placeholders::_1, "vam_ts", &asn_DEF_vam_ts_VAM, std::function<void(const vam_ts_msgs::VAM&, vam_ts_VAM_t&)>(etsi_its_vam_ts_conversion::toStruct_VAM));
    subscribers_["vam_ts"] = this->create_subscription<vam_ts_msgs::VAM>(kInputTopicVamTs, subscriber_queue_size_, callback, subscriber_options);
    RCLCPP_INFO(this->get_logger(), "Converting native ROS VAM (TS) on '%s' to UDP messages on '%s'", subscribers_["vam_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS VAM (TS) to UDP messages via service '%s'", convert_vam_ts_to_udp_service_->get_service_name());
  }
}


template <typename T_struct>
bool Converter::decodeBufferToStruct(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, T_struct* asn1_struct) const {

  asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, (void **)&asn1_struct, buffer, size);
  if (ret.code != RC_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to decode message");
    return false;
  }
  if (logLevelIsDebug()) asn_fprint(stdout, type_descriptor, asn1_struct);

  return true;
}


template <typename T_ros, typename T_struct>
T_ros Converter::structToRosMessage(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn) const {

  T_ros msg;
  conversion_fn(asn1_struct, msg);

  return msg;
}


template <typename T_ros, typename T_struct>
bool Converter::decodeBufferToRosMessage(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn, T_ros& msg) const {

  T_struct asn1_struct{};
  bool success = this->decodeBufferToStruct(buffer, size, type_descriptor, &asn1_struct);
  if (success) msg = this->structToRosMessage(asn1_struct, type_descriptor, conversion_fn);
  ASN_STRUCT_FREE_CONTENTS_ONLY(*type_descriptor, &asn1_struct);

  return success;
}


template <typename T_ros, typename T_struct>
T_struct Converter::rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) const {

  T_struct asn1_struct{};
  conversion_fn(msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, type_descriptor, &asn1_struct);

  return asn1_struct;
}


template <typename T_struct>
bool Converter::encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t*& buffer, int& size) const {

  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(type_descriptor, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Check of struct failed: %s", error_buffer);
    return false;
  }

  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, &asn1_struct);
  if (ret.result.encoded == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return false;
  }

  buffer = static_cast<uint8_t*>(ret.buffer);
  size = ret.result.encoded;

  return true;
}


UdpPacket Converter::bufferToUdpPacketMessage(const uint8_t* buffer, const int size, const int btp_header_destination_port) const {

  UdpPacket udp_msg;

  // add BTP destination port and destination port info
  uint16_t destination_port = htons(btp_header_destination_port);
  udp_msg.src_port = destination_port;
  if (has_btp_destination_port_) {
    uint16_t destination_port_info = 0;
    uint16_t* btp_header = new uint16_t[2] {destination_port, destination_port_info};
    uint8_t* btp_header_uint8 = reinterpret_cast<uint8_t*>(btp_header);
    udp_msg.data.insert(udp_msg.data.end(), btp_header_uint8, btp_header_uint8 + 2 * sizeof(uint16_t));
    delete[] btp_header;
  }

  udp_msg.data.insert(udp_msg.data.end(), buffer, buffer + size);

  return udp_msg;
}

template <typename T_ros, typename T_struct>
bool Converter::encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn, const int btp_header_destination_port) const {

  // convert ROS msg to struct
  auto asn1_struct = this->rosMessageToStruct(msg, type_descriptor, conversion_fn);

  // encode struct to ASN1 bitstring
  uint8_t* buffer = nullptr;
  int buffer_size;
  bool successful = this->encodeStructToBuffer(asn1_struct, type_descriptor, buffer, buffer_size);
  if (!successful) return false;

  // copy bitstring to ROS UDP msg
  udp_msg = this->bufferToUdpPacketMessage(buffer, buffer_size, btp_header_destination_port);

  // free memory
  ASN_STRUCT_FREE_CONTENTS_ONLY(*type_descriptor, &asn1_struct);
  free(buffer);

  return true;
}

template <typename T_ros, typename T_struct, typename T_srv>
void Converter::rosToUdpSrvCallback(const std::shared_ptr<typename T_srv::Request> request,
                          std::shared_ptr<typename T_srv::Response> response, const std::string& type,
                          const asn_TYPE_descriptor_t* asn_type_descriptor,
                          std::function<void(const T_ros&, T_struct&)> conversion_fn) const {

  RCLCPP_INFO(this->get_logger(), "Received service request to convert ETSI message of type '%s' from ROS message to bitstring", type.c_str());
  const auto &msg = request->ros_msg;

  int btp_header_destination_port = 0;
  if (type == "cam" || type == "cam_ts") btp_header_destination_port = kBtpHeaderDestinationPortCam;
  else if (type == "cpm_ts") btp_header_destination_port = kBtpHeaderDestinationPortCpmTs;
  else if (type == "denm" || type == "denm_ts") btp_header_destination_port = kBtpHeaderDestinationPortDenm;
  else if (type == "mapem_ts") btp_header_destination_port = kBtpHeaderDestinationPortMapem;
  else if (type == "mcm_uulm") btp_header_destination_port = kBtpHeaderDestinationPortMcmUulm;
  else if (type == "spatem_ts") btp_header_destination_port = kBtpHeaderDestinationPortSpatem;
  else if (type == "vam_ts") btp_header_destination_port = kBtpHeaderDestinationPortVamTs;

  // encode ROS msg to UDP msg
  UdpPacket udp_msg;
  bool success = this->encodeRosMessageToUdpPacketMessage<T_ros, T_struct>(msg, udp_msg, asn_type_descriptor, conversion_fn, btp_header_destination_port);
  if (!success) return;

  // return result
  response->udp_packet = udp_msg;
  int msg_size = has_btp_destination_port_ ? udp_msg.data.size() - 4 : udp_msg.data.size();
  RCLCPP_INFO(this->get_logger(), "Returned service result for ETSI message of type '%s' as bitstring (message size: %d | total payload size: %ld)", type.c_str(), msg_size, udp_msg.data.size());
}

template <typename T_ros, typename T_struct, typename T_srv>
void Converter::udpToRosSrvCallback(const std::shared_ptr<typename T_srv::Request> request, std::shared_ptr<typename T_srv::Response> response, const std::string& type, const asn_TYPE_descriptor_t* asn_type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn) const {

  const UdpPacket* udp_msg = &request->udp_packet;
  RCLCPP_INFO(this->get_logger(), "Received service request to convert bitstring (total payload size: %ld) to ROS ETSI message", udp_msg->data.size());
  if (udp_msg->data.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Received empty bitstring payload, dropping");
    return;
  }

  // determine message size
  int msg_size = udp_msg->data.size() - etsi_message_payload_offset_;
  if (msg_size <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Payload too short for ETSI message payload (offset %d), dropping", etsi_message_payload_offset_);
    return;
  }

  // decode buffer to ROS msg
  T_ros ros_msg;
  bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, asn_type_descriptor, conversion_fn, ros_msg);
  if (!success) {
    RCLCPP_WARN(this->get_logger(), "Failed to decode UDP packet to ROS message");
    return;
  }

  // return result
  response->ros_msg = ros_msg;
  RCLCPP_INFO(this->get_logger(), "Returned service result for ETSI message of type '%s' as ROS message", type.c_str());
}

void Converter::udpCallback(const UdpPacket::UniquePtr udp_msg) const {

  RCLCPP_DEBUG(this->get_logger(), "Received bitstring (total payload size: %ld)", udp_msg->data.size());
  if (udp_msg->data.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Received empty bitstring payload, dropping");
    return;
  }

  // auto-detect ETSI message type if BTP destination port is present
  std::string detected_etsi_type = udp2ros_etsi_types_.empty() ? "unknown" : udp2ros_etsi_types_[0];
  uint16_t destination_port;
  if (has_btp_destination_port_) {
    if (udp_msg->data.size() < btp_destination_port_offset_ + sizeof(uint16_t)) {
      RCLCPP_ERROR(this->get_logger(), "Payload too short for BTP destination port (need %ld bytes), dropping", btp_destination_port_offset_ + sizeof(uint16_t));
      return;
    }
    const uint16_t* btp_destination_port = reinterpret_cast<const uint16_t*>(&udp_msg->data[btp_destination_port_offset_]);
    destination_port = ntohs(*btp_destination_port);
  } else if (udp_msg->src_port != 0) {
    destination_port = udp_msg->src_port;
  }
  if (destination_port == kBtpHeaderDestinationPortCam) detected_etsi_type = "cam";
  else if (destination_port == kBtpHeaderDestinationPortCpmTs) detected_etsi_type = "cpm_ts";
  else if (destination_port == kBtpHeaderDestinationPortDenm) detected_etsi_type = "denm";
  else if (destination_port == kBtpHeaderDestinationPortIvi) detected_etsi_type = "ivi";
  else if (destination_port == kBtpHeaderDestinationPortMapem) detected_etsi_type = "mapem_ts";
  else if (destination_port == kBtpHeaderDestinationPortMcmUulm) detected_etsi_type = "mcm_uulm";
  else if (destination_port == kBtpHeaderDestinationPortSpatem) detected_etsi_type = "spatem_ts";
  else if (destination_port == kBtpHeaderDestinationPortVamTs) detected_etsi_type = "vam_ts";
  else detected_etsi_type = "unknown";


  int msg_size = udp_msg->data.size() - etsi_message_payload_offset_;
  if (msg_size <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Payload too short for ETSI message payload (offset %d), dropping", etsi_message_payload_offset_);
    return;
  }
  const uint8_t* protocol_version = reinterpret_cast<const uint8_t*>(&udp_msg->data[etsi_message_payload_offset_]);
  RCLCPP_INFO(this->get_logger(), "Received ETSI message of type '%s' (protocolVersion: %d) as bitstring (message size: %d | total payload size: %ld)", detected_etsi_type.c_str(), *protocol_version , msg_size, udp_msg->data.size());

  if (detected_etsi_type == "cam" || detected_etsi_type == "cam_ts") {

    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam") != udp2ros_etsi_types_.end()) { // CAM EN v1.4.1
      cam_msgs::CAM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cam_CAM, std::function<void(const cam_CAM_t&, cam_msgs::CAM&)>(etsi_its_cam_conversion::toRos_CAM), msg);
      if (!success) return;
      publisher_cam_->publish(msg);
    }
    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam_ts") != udp2ros_etsi_types_.end()) { // CAM TS v2.1.1
      cam_ts_msgs::CAM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cam_ts_CAM, std::function<void(const cam_ts_CAM_t&, cam_ts_msgs::CAM&)>(etsi_its_cam_ts_conversion::toRos_CAM), msg);
      if (!success) return;
      publisher_cam_ts_->publish(msg);
    }

  } else if (detected_etsi_type == "cpm_ts") {

    // decode buffer to ROS msg
    cpm_ts_msgs::CollectivePerceptionMessage msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cpm_ts_CollectivePerceptionMessage, std::function<void(const cpm_ts_CollectivePerceptionMessage_t&, cpm_ts_msgs::CollectivePerceptionMessage&)>(etsi_its_cpm_ts_conversion::toRos_CollectivePerceptionMessage), msg);
    if (!success) return;

    // publish msg
    publisher_cpm_ts_->publish(msg);

  } else if (detected_etsi_type == "denm" || detected_etsi_type == "denm_ts") {

    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm") != udp2ros_etsi_types_.end()) { // DENM EN v1.3.1
      denm_msgs::DENM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_denm_DENM, std::function<void(const denm_DENM_t&, denm_msgs::DENM&)>(etsi_its_denm_conversion::toRos_DENM), msg);
      if (!success) return;
      publisher_denm_->publish(msg);
    }
    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm_ts") != udp2ros_etsi_types_.end()) { // DENM TS v2.2.1
      denm_ts_msgs::DENM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_denm_ts_DENM, std::function<void(const denm_ts_DENM_t&, denm_ts_msgs::DENM&)>(etsi_its_denm_ts_conversion::toRos_DENM), msg);
      if (!success) return;
      publisher_denm_ts_->publish(msg);
    }

  } else if (detected_etsi_type == "vam_ts") {

    // decode buffer to ROS msg
    vam_ts_msgs::VAM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_vam_ts_VAM, std::function<void(const vam_ts_VAM_t&, vam_ts_msgs::VAM&)>(etsi_its_vam_ts_conversion::toRos_VAM), msg);
    if (!success) return;

    // publish msg
    publisher_vam_ts_->publish(msg);

  } else if (detected_etsi_type == "mapem_ts") {

    // decode buffer to ROS msg
    mapem_ts_msgs::MAPEM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_mapem_ts_MAPEM, std::function<void(const mapem_ts_MAPEM_t&, mapem_ts_msgs::MAPEM&)>(etsi_its_mapem_ts_conversion::toRos_MAPEM), msg);
    if (!success) return;

    // publish msg
    publisher_mapem_ts_->publish(msg);

  } else if (detected_etsi_type == "mcm_uulm") {

    // decode buffer to ROS msg
    mcm_uulm_msgs::MCM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_mcm_uulm_MCM, std::function<void(const mcm_uulm_MCM_t&, mcm_uulm_msgs::MCM&)>(etsi_its_mcm_uulm_conversion::toRos_MCM), msg);
    if (!success) return;

    // publish msg
    publisher_mcm_uulm_->publish(msg);

  } else if (detected_etsi_type == "spatem_ts") {

    // decode buffer to ROS msg
    spatem_ts_msgs::SPATEM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_spatem_ts_SPATEM, std::function<void(const spatem_ts_SPATEM_t&, spatem_ts_msgs::SPATEM&)>(etsi_its_spatem_ts_conversion::toRos_SPATEM), msg);
    if (!success) return;

    // publish msg
    publisher_spatem_ts_->publish(msg);

  } else {
    RCLCPP_ERROR(this->get_logger(), "Detected ETSI message type '%s' not yet supported, dropping message", detected_etsi_type.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Published ETSI message of type '%s' as ROS message", detected_etsi_type.c_str());
}


template <typename T_ros, typename T_struct>
void Converter::rosCallback(const typename T_ros::UniquePtr msg,
                            const std::string& type, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) const {

  RCLCPP_INFO(this->get_logger(), "Received ETSI message of type '%s' as ROS message", type.c_str());

  int btp_header_destination_port = 0;
  if (type == "cam" || type == "cam_ts") btp_header_destination_port = kBtpHeaderDestinationPortCam;
  else if (type == "cpm_ts") btp_header_destination_port = kBtpHeaderDestinationPortCpmTs;
  else if (type == "denm" || type == "denm_ts") btp_header_destination_port = kBtpHeaderDestinationPortDenm;
  else if (type == "mapem_ts") btp_header_destination_port = kBtpHeaderDestinationPortMapem;
  else if (type == "mcm_uulm") btp_header_destination_port = kBtpHeaderDestinationPortMcmUulm;
  else if (type == "spatem_ts") btp_header_destination_port = kBtpHeaderDestinationPortSpatem;
  else if (type == "vam_ts") btp_header_destination_port = kBtpHeaderDestinationPortVamTs;

  // encode ROS msg to UDP msg
  UdpPacket udp_msg;
  bool success = this->encodeRosMessageToUdpPacketMessage<T_ros, T_struct>(*msg, udp_msg, type_descriptor, conversion_fn, btp_header_destination_port);
  if (!success) return;

  // publish UDP msg
  publisher_udp_->publish(udp_msg);
  int msg_size = has_btp_destination_port_ ? udp_msg.data.size() - 4 : udp_msg.data.size();
  RCLCPP_INFO(this->get_logger(), "Published ETSI message of type '%s' as bitstring (message size: %d | total payload size: %ld)", type.c_str(), msg_size, udp_msg.data.size());
}


}  // end of namespace
