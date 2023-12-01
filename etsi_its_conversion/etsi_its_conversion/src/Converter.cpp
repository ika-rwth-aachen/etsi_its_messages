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
const int kBtpHeaderDestinationPortMapem{2003};
const int kBtpHeaderDestinationPortSpatem{2004};
const int kBtpHeaderDestinationPortIvi{2006};
const int kBtpHeaderDestinationPortCpm{2009};

#ifdef ROS1
const std::string Converter::kInputTopicUdp{"udp/in"};
const std::string Converter::kOutputTopicUdp{"udp/out"};
const std::string Converter::kInputTopicCam{"cam/in"};
const std::string Converter::kOutputTopicCam{"cam/out"};
const std::string Converter::kInputTopicDenm{"denm/in"};
const std::string Converter::kOutputTopicDenm{"denm/out"};
const std::string Converter::kInputTopicSpatem{"spatem/in"};
const std::string Converter::kOutputTopicSpatem{"spatem/out"};
const std::string Converter::kInputTopicMapem{"mapem/in"};
const std::string Converter::kOutputTopicMapem{"mapem/out"};
#else
const std::string Converter::kInputTopicUdp{"~/udp/in"};
const std::string Converter::kOutputTopicUdp{"~/udp/out"};
const std::string Converter::kInputTopicCam{"~/cam/in"};
const std::string Converter::kOutputTopicCam{"~/cam/out"};
const std::string Converter::kInputTopicDenm{"~/denm/in"};
const std::string Converter::kOutputTopicDenm{"~/denm/out"};
const std::string Converter::kInputTopicSpatem{"~/spatem/in"};
const std::string Converter::kOutputTopicSpatem{"~/spatem/out"};
const std::string Converter::kInputTopicMapem{"~/mapem/in"};
const std::string Converter::kOutputTopicMapem{"~/mapem/out"};
#endif

const std::string Converter::kEtsiTypeParam{"etsi_type"};
const std::string Converter::kEtsiTypeParamDefault{"auto"};


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

  std::vector<std::string> known_etsi_types = {kEtsiTypeParamDefault, "cam", "denm", "spatem", "mapem"};

#ifdef ROS1
  if (!private_node_handle_.param<std::string>(kEtsiTypeParam, etsi_type_, kEtsiTypeParamDefault)) {
    NODELET_WARN(
#else
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  std::stringstream ss;
  ss << "ETSI type to convert, one of ";
  for (const auto& e : known_etsi_types) ss << e << ", ";
  param_desc.description = ss.str();
  this->declare_parameter(kEtsiTypeParam, kEtsiTypeParamDefault, param_desc);

  if (!this->get_parameter(kEtsiTypeParam, etsi_type_)) {
    RCLCPP_WARN(this->get_logger(),
#endif
      "Parameter '%s' is not set, defaulting to '%s'", kEtsiTypeParam.c_str(), kEtsiTypeParamDefault.c_str());
  }

  if (std::find(known_etsi_types.begin(), known_etsi_types.end(), etsi_type_) == known_etsi_types.end()) {
#ifdef ROS1
    NODELET_WARN(
#else
    RCLCPP_WARN(this->get_logger(),
#endif
      "Invalid value for parameter '%s', defaulting to '%s'", kEtsiTypeParam.c_str(), kEtsiTypeParamDefault.c_str());
  }
}


void Converter::setup() {

  // create subscribers and publishers
#ifdef ROS1
  publisher_udp_ = private_node_handle_.advertise<udp_msgs::UdpPacket>(kOutputTopicUdp, 1);
  publishers_["cam"] = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>(kOutputTopicCam, 1);
  publishers_["denm"] = private_node_handle_.advertise<etsi_its_denm_msgs::DENM>(kOutputTopicDenm, 1);
  publishers_["spatem"] = private_node_handle_.advertise<etsi_its_spatem_msgs::SPATEM>(kOutputTopicSpatem, 1);
  publishers_["mapem"] = private_node_handle_.advertise<etsi_its_mapem_msgs::MAPEM>(kOutputTopicMapem, 1);
  subscriber_udp_ = private_node_handle_.subscribe(kInputTopicUdp, 1, &Converter::udpCallback, this);
  subscribers_["cam"] = private_node_handle_.subscribe(kInputTopicCam, 1, &Converter::rosCallbackCam, this);
  subscribers_["denm"] = private_node_handle_.subscribe(kInputTopicDenm, 1, &Converter::rosCallbackDenm, this);
  subscribers_["spatem"] = private_node_handle_.subscribe(kInputTopicSpatem, 1, &Converter::rosCallbackSpatem, this);
  subscribers_["mapem"] = private_node_handle_.subscribe(kInputTopicMapem, 1, &Converter::rosCallbackMapem, this);
  NODELET_INFO("Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_.getTopic().c_str(), publishers_["cam"].getTopic().c_str());
  NODELET_INFO("Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
  NODELET_INFO("Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_.getTopic().c_str(), publishers_["denm"].getTopic().c_str());
  NODELET_INFO("Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
  NODELET_INFO("Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_.getTopic().c_str(), publishers_["spatem"].getTopic().c_str());
  NODELET_INFO("Converting native ROS SPATEMs on '%s' to UDP messages on '%s'", subscribers_["spatem"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
  NODELET_INFO("Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_.getTopic().c_str(), publishers_["mapem"].getTopic().c_str());
  NODELET_INFO("Converting native ROS MAPEMs on '%s' to UDP messages on '%s'", subscribers_["mapem"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
#else
  publisher_udp_ = this->create_publisher<udp_msgs::msg::UdpPacket>(kOutputTopicUdp, 1);
  publishers_cam_["cam"] = this->create_publisher<etsi_its_cam_msgs::msg::CAM>(kOutputTopicCam, 1);
  publishers_denm_["denm"] = this->create_publisher<etsi_its_denm_msgs::msg::DENM>(kOutputTopicDenm, 1);
  publishers_spatem_["spatem"] = this->create_publisher<etsi_its_spatem_msgs::msg::SPATEM>(kOutputTopicSpatem, 1);
  publishers_mapem_["mapem"] = this->create_publisher<etsi_its_mapem_msgs::msg::MAPEM>(kOutputTopicMapem, 1);
  subscriber_udp_ = this->create_subscription<udp_msgs::msg::UdpPacket>(kInputTopicUdp, 1, std::bind(&Converter::udpCallback, this, std::placeholders::_1));
  subscribers_cam_["cam"] = this->create_subscription<etsi_its_cam_msgs::msg::CAM>(kInputTopicCam, 1, std::bind(&Converter::rosCallbackCam, this, std::placeholders::_1));
  subscribers_denm_["denm"] = this->create_subscription<etsi_its_denm_msgs::msg::DENM>(kInputTopicDenm, 1, std::bind(&Converter::rosCallbackDenm, this, std::placeholders::_1));
  subscribers_spatem_["spatem"] = this->create_subscription<etsi_its_spatem_msgs::msg::SPATEM>(kInputTopicSpatem, 1, std::bind(&Converter::rosCallbackSpatem, this, std::placeholders::_1));
  subscribers_mapem_["mapem"] = this->create_subscription<etsi_its_mapem_msgs::msg::MAPEM>(kInputTopicMapem, 1, std::bind(&Converter::rosCallbackMapem, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_->get_topic_name(), publishers_cam_["cam"]->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_cam_["cam"]->get_topic_name(), publisher_udp_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_->get_topic_name(), publishers_denm_["denm"]->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_denm_["denm"]->get_topic_name(), publisher_udp_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_->get_topic_name(), publishers_spatem_["spatem"]->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting native ROS SPATEMs on '%s' to UDP messages on '%s'", subscribers_spatem_["spatem"]->get_topic_name(), publisher_udp_->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_->get_topic_name(), publishers_mapem_["mapem"]->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Converting native ROS MAPEMs on '%s' to UDP messages on '%s'", subscribers_mapem_["mapem"]->get_topic_name(), publisher_udp_->get_topic_name());
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

  // decode BTP-Header, if type detection is enabled
  std::string detected_etsi_type = etsi_type_;
  int offset = 0;
  if (etsi_type_ == "auto") {
    offset = 4;
    const uint16_t* btp_header = reinterpret_cast<const uint16_t*>(&udp_msg->data[0]);
    uint16_t destination_port = ntohs(btp_header[0]);
    if (destination_port == kBtpHeaderDestinationPortCam) detected_etsi_type = "cam";
    else if (destination_port == kBtpHeaderDestinationPortDenm) detected_etsi_type = "denm";
    else if (destination_port == kBtpHeaderDestinationPortMapem) detected_etsi_type = "mapem";
    else if (destination_port == kBtpHeaderDestinationPortSpatem) detected_etsi_type = "spatem";
    else if (destination_port == kBtpHeaderDestinationPortMapem) detected_etsi_type = "mapem";
    else if (destination_port == kBtpHeaderDestinationPortIvi) detected_etsi_type = "ivi";
    else if (destination_port == kBtpHeaderDestinationPortCpm) detected_etsi_type = "cpm";
    else detected_etsi_type = "unknown";
  }

  if (detected_etsi_type == "cam") {

    // decode ASN1 bitstring to struct
    CAM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&asn1_struct, &udp_msg->data[offset], udp_msg->data.size() - offset);
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
    NODELET_DEBUG(
#else
    publishers_cam_["cam"]->publish(msg);
    RCLCPP_DEBUG(this->get_logger(),
#endif
      "Published CAM");

  } else if (detected_etsi_type == "denm") {

    // decode ASN1 bitstring to struct
    DENM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_DENM, (void **)&asn1_struct, &udp_msg->data[offset], udp_msg->data.size() - offset);
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
    NODELET_DEBUG(
#else
    publishers_denm_["denm"]->publish(msg);
    RCLCPP_DEBUG(this->get_logger(),
#endif
      "Published DENM");

  } else if (detected_etsi_type == "spatem") {

    // decode ASN1 bitstring to struct
    SPATEM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_SPATEM, (void **)&asn1_struct, &udp_msg->data[offset], udp_msg->data.size() - offset);
    if (ret.code != RC_OK) {
#ifdef ROS1
      NODELET_ERROR(
#else
      RCLCPP_ERROR(this->get_logger(),
#endif
        "Failed to decode message");
      return;
    }
    if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_SPATEM, asn1_struct);

    // convert struct to ROS msg and publish
#ifdef ROS1
    etsi_its_spatem_msgs::SPATEM msg;
#else
    etsi_its_spatem_msgs::msg::SPATEM msg;
#endif
    etsi_its_spatem_conversion::toRos_SPATEM(*asn1_struct, msg);

    // publish msg
#ifdef ROS1
    publishers_["spatem"].publish(msg);
    NODELET_DEBUG(
#else
    publishers_spatem_["spatem"]->publish(msg);
    RCLCPP_DEBUG(this->get_logger(),
#endif
      "Published SPATEM");

  } else if (detected_etsi_type == "mapem") {

    // decode ASN1 bitstring to struct
    MAPEM_t* asn1_struct = nullptr;
    asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_MAPEM, (void **)&asn1_struct, &udp_msg->data[offset], udp_msg->data.size() - offset);
    if (ret.code != RC_OK) {
#ifdef ROS1
      NODELET_ERROR(
#else
      RCLCPP_ERROR(this->get_logger(),
#endif
        "Failed to decode message");
      return;
    }
    if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_MAPEM, asn1_struct);

    // convert struct to ROS msg and publish
#ifdef ROS1
    etsi_its_mapem_msgs::MAPEM msg;
#else
    etsi_its_mapem_msgs::msg::MAPEM msg;
#endif
    etsi_its_mapem_conversion::toRos_MAPEM(*asn1_struct, msg);

    // publish msg
#ifdef ROS1
    publishers_["mapem"].publish(msg);
    NODELET_DEBUG(
#else
    publishers_mapem_["mapem"]->publish(msg);
    RCLCPP_DEBUG(this->get_logger(),
#endif
      "Published MAPEM");

  } else {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Detected ETSI message type '%s' not yet supported, dropping message", detected_etsi_type.c_str());
    return;
  }
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
  if (etsi_type_ == "auto") {
    // add BTP-Header, if type detection is enabled
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
  if (etsi_type_ == "auto") {
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


#ifdef ROS1
void Converter::rosCallbackSpatem(const etsi_its_spatem_msgs::SPATEM::ConstPtr msg) {
#else
void Converter::rosCallbackSpatem(const etsi_its_spatem_msgs::msg::SPATEM::UniquePtr msg) {
#endif

#ifdef ROS1
  NODELET_DEBUG(
#else
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Received SPATEM");

  // convert ROS msg to struct
  SPATEM_t asn1_struct;
  etsi_its_spatem_conversion::toStruct_SPATEM(*msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_SPATEM, &asn1_struct);

  // encode struct to ASN1 bitstring
  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(&asn_DEF_SPATEM, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Check of struct failed: %s", error_buffer);
    return;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_SPATEM, &asn1_struct);
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
  if (etsi_type_ == "auto") {
    // add BTP-Header, if type detection is enabled
    uint16_t destination_port = htons(kBtpHeaderDestinationPortSpatem);
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
    "Published SPATEM bitstring");
}


#ifdef ROS1
void Converter::rosCallbackMapem(const etsi_its_mapem_msgs::MAPEM::ConstPtr msg) {
#else
void Converter::rosCallbackMapem(const etsi_its_mapem_msgs::msg::MAPEM::UniquePtr msg) {
#endif

#ifdef ROS1
  NODELET_DEBUG(
#else
  RCLCPP_DEBUG(this->get_logger(),
#endif
    "Received MAPEM");

  // convert ROS msg to struct
  MAPEM_t asn1_struct;
  etsi_its_mapem_conversion::toStruct_MAPEM(*msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_MAPEM, &asn1_struct);

  // encode struct to ASN1 bitstring
  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(&asn_DEF_MAPEM, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
#ifdef ROS1
    NODELET_ERROR(
#else
    RCLCPP_ERROR(this->get_logger(),
#endif
      "Check of struct failed: %s", error_buffer);
    return;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_MAPEM, &asn1_struct);
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
  if (etsi_type_ == "auto") {
    // add BTP-Header, if type detection is enabled
    uint16_t destination_port = htons(kBtpHeaderDestinationPortMapem);
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
    "Published MAPEM bitstring");
}


}  // end of namespace
