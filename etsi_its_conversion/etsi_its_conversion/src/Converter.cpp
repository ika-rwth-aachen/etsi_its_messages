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

#include <algorithm>
#include <arpa/inet.h>
#include <sstream>

#ifdef ROS1
#include <ros/console.h>
#else
#include <rcutils/logging.h>
#endif

#include "etsi_its_conversion/Converter.hpp"

#ifdef ROS1
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(etsi_its_conversion::Converter, nodelet::Nodelet)
#else
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(etsi_its_conversion::Converter)
#endif


namespace etsi_its_conversion {

const int kBtpHeaderDestinationPortCam{2001};
const int kBtpHeaderDestinationPortDenm{2002};
const int kBtpHeaderDestinationPortMap{2003};
const int kBtpHeaderDestinationPortSpat{2004};
const int kBtpHeaderDestinationPortIvi{2006};
const int kBtpHeaderDestinationPortCpm{2009};

#ifdef ROS1
const std::string Converter::kInputTopicUdp{"udp/in"};
const std::string Converter::kOutputTopicUdp{"udp/out"};
const std::string Converter::kInputTopicCam{"cam/in"};
const std::string Converter::kOutputTopicCam{"cam/out"};
const std::string Converter::kInputTopicDenm{"denm/in"};
const std::string Converter::kOutputTopicDenm{"denm/out"};
#else
const std::string Converter::kInputTopicUdp{"~/udp/in"};
const std::string Converter::kOutputTopicUdp{"~/udp/out"};
const std::string Converter::kInputTopicCam{"~/cam/in"};
const std::string Converter::kOutputTopicCam{"~/cam/out"};
const std::string Converter::kInputTopicDenm{"~/denm/in"};
const std::string Converter::kOutputTopicDenm{"~/denm/out"};
#endif

const std::string Converter::kHasBtpDestinationPortParam{"has_btp_destination_port"};
const bool Converter::kHasBtpDestinationPortParamDefault{true};
const std::string Converter::kBtpDestinationPortOffsetParam{"btp_destination_port_offset"};
const int Converter::kBtpDestinationPortOffsetParamDefault{8};
const std::string Converter::kEtsiMessagePayloadOffsetParam{"etsi_message_payload_offset"};
const int Converter::kEtsiMessagePayloadOffsetParamDefault{78};
const std::string Converter::kEtsiTypesParam{"etsi_types"};
const std::vector<std::string> Converter::kEtsiTypesParamDefault{"cam", "denm"};


bool Converter::logLevelIsDebug() {

#ifdef ROS1
  std::map<std::string, ros::console::levels::Level> loggers;
  bool ret = ros::console::get_loggers(loggers);
  std::string node_logger = ROSCONSOLE_DEFAULT_NAME;
  node_logger += "." + ros::this_node::getName();
  if (loggers.count(node_logger) > 0) {
    if (loggers[node_logger] == ros::console::levels::Level::Debug) return true;
  }
  std::string nodelet_logger = "ros.nodelet." + ros::this_node::getName();
  if (loggers.count(nodelet_logger) > 0) {
    if (loggers[nodelet_logger] == ros::console::levels::Level::Debug) return true;
  }
#else
  auto logger_level = rcutils_logging_get_logger_effective_level(this->get_logger().get_name());
  return (logger_level == RCUTILS_LOG_SEVERITY_DEBUG);
#endif

  return false;
}


#ifdef ROS1
void Converter::onInit() {

  private_node_handle_ = this->getMTPrivateNodeHandle();
#else
Converter::Converter(const rclcpp::NodeOptions& options) : Node("converter", options) {
#endif

  this->loadParameters();
  this->setup();
}


void Converter::loadParameters() {

#ifndef ROS1
  rcl_interfaces::msg::ParameterDescriptor param_desc;
#endif

  // load has_btp_destination_port
#ifdef ROS1
  if (!private_node_handle_.param<bool>(kHasBtpDestinationPortParam, has_btp_destination_port_, kHasBtpDestinationPortParamDefault)) {
    NODELET_WARN(
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "whether incoming/outgoing UDP messages include a 4-byte BTP header";
  this->declare_parameter(kHasBtpDestinationPortParam, kHasBtpDestinationPortParamDefault, param_desc);
  if (!this->get_parameter(kHasBtpDestinationPortParam, has_btp_destination_port_)) {
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is not set, defaulting to '%s'", kHasBtpDestinationPortParam.c_str(), kHasBtpDestinationPortParamDefault ? "true" : "false");
  }

  // load btp_destination_port_offset
#ifdef ROS1
  if (!private_node_handle_.param<int>(kBtpDestinationPortOffsetParam, btp_destination_port_offset_, kBtpDestinationPortOffsetParamDefault)) {
    NODELET_WARN(
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kBtpDestinationPortOffsetParam, kBtpDestinationPortOffsetParamDefault, param_desc);
  if (!this->get_parameter(kBtpDestinationPortOffsetParam, btp_destination_port_offset_)) {
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is not set, defaulting to '%d'", kBtpDestinationPortOffsetParam.c_str(), kBtpDestinationPortOffsetParamDefault);
  }

  // load etsi_message_payload_offset
#ifdef ROS1
  if (!private_node_handle_.param<int>(kEtsiMessagePayloadOffsetParam, etsi_message_payload_offset_, kEtsiMessagePayloadOffsetParamDefault)) {
    NODELET_WARN(
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kEtsiMessagePayloadOffsetParam, kEtsiMessagePayloadOffsetParamDefault, param_desc);
  if (!this->get_parameter(kEtsiMessagePayloadOffsetParam, etsi_message_payload_offset_)) {
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is not set, defaulting to '%d'", kEtsiMessagePayloadOffsetParam.c_str(), kEtsiMessagePayloadOffsetParamDefault);
  }

  // load etsi_types
#ifdef ROS1
  if (!private_node_handle_.param<std::vector<std::string>>(kEtsiTypesParam, etsi_types_, kEtsiTypesParamDefault)) {
    NODELET_WARN(
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss;
  ss << "list of ETSI types to convert, one of ";
  for (const auto& e : kEtsiTypesParamDefault) ss << e << ", ";
  param_desc.description = ss.str();
  this->declare_parameter(kEtsiTypesParam, kEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kEtsiTypesParam, etsi_types_)) {
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is not set, defaulting to all", kEtsiTypesParam.c_str());
  }

  // check etsi_types
  for (const auto& e : etsi_types_) {
    if (std::find(kEtsiTypesParamDefault.begin(), kEtsiTypesParamDefault.end(), e) == kEtsiTypesParamDefault.end())
#ifdef ROS1
      NODELET_WARN(
#else
      RCLCPP_WARN(this->get_logger(),
#endif
        "Invalid value '%s' for parameter '%s', skipping", e.c_str(), kEtsiTypesParam.c_str());
  }
  if (!has_btp_destination_port_ && etsi_types_.size() > 1) {
#ifdef ROS1
    NODELET_WARN(
#else
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is set to 'false', but multiple 'etsi_types' are configured, dropping all but the first", kHasBtpDestinationPortParam.c_str());
    etsi_types_.resize(1);
  }
}


void Converter::setup() {

  // create subscribers and publishers
#ifdef ROS1
  publisher_udp_ = private_node_handle_.advertise<udp_msgs::UdpPacket>(kOutputTopicUdp, 1);
  publishers_["cam"] = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>(kOutputTopicCam, 1);
  publishers_["denm"] = private_node_handle_.advertise<etsi_its_denm_msgs::DENM>(kOutputTopicDenm, 1);
  subscriber_udp_ = private_node_handle_.subscribe(kInputTopicUdp, 1, &Converter::udpCallback, this);
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "cam") != etsi_types_.end()) {
    subscribers_["cam"] = private_node_handle_.subscribe(kInputTopicCam, 1, &Converter::rosCallbackCam, this);
    NODELET_INFO("Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_.getTopic().c_str(), publishers_["cam"].getTopic().c_str());
    NODELET_INFO("Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "denm") != etsi_types_.end()) {
    subscribers_["denm"] = private_node_handle_.subscribe(kInputTopicDenm, 1, &Converter::rosCallbackDenm, this);
    NODELET_INFO("Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_.getTopic().c_str(), publishers_["denm"].getTopic().c_str());
    NODELET_INFO("Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
  }
#else
  publisher_udp_ = this->create_publisher<udp_msgs::msg::UdpPacket>(kOutputTopicUdp, 1);
  publishers_cam_["cam"] = this->create_publisher<etsi_its_cam_msgs::msg::CAM>(kOutputTopicCam, 1);
  publishers_denm_["denm"] = this->create_publisher<etsi_its_denm_msgs::msg::DENM>(kOutputTopicDenm, 1);
  subscriber_udp_ = this->create_subscription<udp_msgs::msg::UdpPacket>(kInputTopicUdp, 1, std::bind(&Converter::udpCallback, this, std::placeholders::_1));
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "cam") != etsi_types_.end()) {
    subscribers_cam_["cam"] = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(kInputTopicCam, 1, std::bind(&Converter::rosCallbackCam, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publishers_cam_["cam"]->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_cam_["cam"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "denm") != etsi_types_.end()) {
    subscribers_denm_["denm"] = this->create_subscription<etsi_its_denm_msgs::msg::DENM>(kInputTopicDenm, 1, std::bind(&Converter::rosCallbackDenm, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publishers_denm_["denm"]->get_topic_name());
    RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_denm_["denm"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
#endif
}


#ifdef ROS1
void Converter::udpCallback(const udp_msgs::UdpPacket::ConstPtr udp_msg) {
#else
void Converter::udpCallback(const udp_msgs::msg::UdpPacket::UniquePtr udp_msg) {
#endif

#ifdef ROS1
  NODELET_DEBUG(
#else
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Received bitstring");

  // auto-detect ETSI message type if BTP destination port is present
  std::string detected_etsi_type = etsi_types_[0];
  if (has_btp_destination_port_) {
    const uint16_t* btp_destination_port = reinterpret_cast<const uint16_t*>(&udp_msg->data[btp_destination_port_offset_]);
    uint16_t destination_port = ntohs(*btp_destination_port);
    if (destination_port == kBtpHeaderDestinationPortCam) detected_etsi_type = "cam";
    else if (destination_port == kBtpHeaderDestinationPortDenm) detected_etsi_type = "denm";
    else if (destination_port == kBtpHeaderDestinationPortMap) detected_etsi_type = "map";
    else if (destination_port == kBtpHeaderDestinationPortSpat) detected_etsi_type = "spat";
    else if (destination_port == kBtpHeaderDestinationPortIvi) detected_etsi_type = "ivi";
    else if (destination_port == kBtpHeaderDestinationPortCpm) detected_etsi_type = "cpm";
    else detected_etsi_type = "unknown";
  }

  int msg_size = udp_msg->data.size() - etsi_message_payload_offset_;
#ifdef ROS1
  NODELET_INFO(
#else
  RCLCPP_INFO(this->get_logger(),
#endif
    "Received ETSI message of type '%s' (message size: %d | total payload size: %ld)", detected_etsi_type.c_str(), msg_size, udp_msg->data.size());

  if (detected_etsi_type == "cam") {

    // decode ASN1 bitstring to struct
    CAM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&asn1_struct, &udp_msg->data[etsi_message_payload_offset_], msg_size);
    if (ret.code != RC_OK) {
#ifdef ROS1
      NODELET_ERROR(
#else
      RCLCPP_ERROR(this->get_logger(),
#endif
        "Failed to decode message");
      return;
    }
    if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_CAM, asn1_struct);

    // convert struct to ROS msg and publish
#ifdef ROS1
    etsi_its_cam_msgs::CAM msg;
#else
    etsi_its_cam_msgs::msg::CAM msg;
#endif
    etsi_its_cam_conversion::toRos_CAM(*asn1_struct, msg);

    // publish msg
#ifdef ROS1
    publishers_["cam"].publish(msg);
#else
    publishers_cam_["cam"]->publish(msg);
#endif

  } else if (detected_etsi_type == "denm") {

    // decode ASN1 bitstring to struct
    DENM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, (void **)&asn1_struct, &udp_msg->data[etsi_message_payload_offset_], msg_size);
    if (ret.code != RC_OK) {
#ifdef ROS1
      NODELET_ERROR(
#else
      RCLCPP_ERROR(this->get_logger(),
#endif
        "Failed to decode message");
      return;
    }
    if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_DENM, asn1_struct);

    // convert struct to ROS msg and publish
#ifdef ROS1
    etsi_its_denm_msgs::DENM msg;
#else
    etsi_its_denm_msgs::msg::DENM msg;
#endif
    etsi_its_denm_conversion::toRos_DENM(*asn1_struct, msg);

    // publish msg
#ifdef ROS1
    publishers_["denm"].publish(msg);
#else
    publishers_denm_["denm"]->publish(msg);
#endif

  } else {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Detected ETSI message type '%s' not yet supported, dropping message", detected_etsi_type.c_str());
    return;
  }

#ifdef ROS1
    NODELET_INFO(
#else
    RCLCPP_INFO(this->get_logger(),
#endif
      "Published ETSI message of type '%s' as ROS message", detected_etsi_type.c_str());
}


#ifdef ROS1
void Converter::rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg) {
#else
void Converter::rosCallbackCam(const etsi_its_cam_msgs::msg::CAM::UniquePtr msg) {
#endif

#ifdef ROS1
  NODELET_DEBUG(
#else
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Received CAM");

  // convert ROS msg to struct
  CAM_t asn1_struct;
  etsi_its_cam_conversion::toStruct_CAM(*msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_CAM, &asn1_struct);

  // encode struct to ASN1 bitstring
  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(&asn_DEF_CAM, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Check of struct failed: %s", error_buffer);
    return;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &asn1_struct);
  if (ret.result.encoded == -1) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return;
  }

  // copy bitstring to ROS UDP msg
#ifdef ROS1
  udp_msgs::UdpPacket udp_msg;
#else
  udp_msgs::msg::UdpPacket udp_msg;
#endif
  if (has_btp_destination_port_) {
    // add BTP destination port and destination port info
    uint16_t destination_port = htons(kBtpHeaderDestinationPortCam);
    uint16_t destination_port_info = 0;
    uint16_t* btp_header = new uint16_t[2] {destination_port, destination_port_info};
    uint8_t* btp_header_uint8 = reinterpret_cast<uint8_t*>(btp_header);
    udp_msg.data.insert(udp_msg.data.end(), btp_header_uint8, btp_header_uint8 + 2 * sizeof(uint16_t));
    delete[] btp_header;
  }
  udp_msg.data.insert(udp_msg.data.end(), (uint8_t*)ret.buffer, (uint8_t*)ret.buffer + (int)ret.result.encoded);

  // publish UDP msg
#ifdef ROS1
  publisher_udp_.publish(udp_msg);
  NODELET_DEBUG(
#else
  publisher_udp_->publish(udp_msg);
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Published CAM bitstring");
}


#ifdef ROS1
void Converter::rosCallbackDenm(const etsi_its_denm_msgs::DENM::ConstPtr msg) {
#else
void Converter::rosCallbackDenm(const etsi_its_denm_msgs::msg::DENM::UniquePtr msg) {
#endif

#ifdef ROS1
  NODELET_DEBUG(
#else
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Received DENM");

  // convert ROS msg to struct
  DENM_t asn1_struct;
  etsi_its_denm_conversion::toStruct_DENM(*msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_DENM, &asn1_struct);

  // encode struct to ASN1 bitstring
  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(&asn_DEF_DENM, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Check of struct failed: %s", error_buffer);
    return;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, &asn1_struct);
  if (ret.result.encoded == -1) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return;
  }

  // copy bitstring to ROS UDP msg
#ifdef ROS1
  udp_msgs::UdpPacket udp_msg;
#else
  udp_msgs::msg::UdpPacket udp_msg;
#endif
  if (has_btp_destination_port_) {
    // add BTP-Header, if type detection is enabled
    uint16_t destination_port = htons(kBtpHeaderDestinationPortDenm);
    uint16_t destination_port_info = 0;
    uint16_t* btp_header = new uint16_t[2] {destination_port, destination_port_info};
    uint8_t* btp_header_uint8 = reinterpret_cast<uint8_t*>(btp_header);
    udp_msg.data.insert(udp_msg.data.end(), btp_header_uint8, btp_header_uint8 + 2 * sizeof(uint16_t));
    delete[] btp_header;
  }
  udp_msg.data.insert(udp_msg.data.end(), (uint8_t*)ret.buffer, (uint8_t*)ret.buffer + (int)ret.result.encoded);

  // publish UDP msg
#ifdef ROS1
  publisher_udp_.publish(udp_msg);
  NODELET_DEBUG(
#else
  publisher_udp_->publish(udp_msg);
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Published DENM bitstring");
}


}  // end of namespace
