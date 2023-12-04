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
#define ROS_LOG(level, ...) NODELET_##level(__VA_ARGS__)
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(etsi_its_conversion::Converter, nodelet::Nodelet)
#else
#define ROS_LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)
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

const std::string Converter::kHasBtpHeaderParam{"has_btp_header"};
const bool Converter::kHasBtpHeaderParamDefault{true};
const std::string Converter::kEtsiTypesParam{"etsi_types"};
const std::vector<std::string> Converter::kEtsiTypesParamDefault{{"cam", "denm", "spatem", "mapem"}};


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

  // load has_btp_header
#ifdef ROS1
  if (!private_node_handle_.param<bool>(kHasBtpHeaderParam, has_btp_header_, kHasBtpHeaderParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "whether incoming/outgoing UDP messages include a 4-byte BTP header";
  this->declare_parameter(kHasBtpHeaderParam, kHasBtpHeaderParamDefault, param_desc);
  if (!this->get_parameter(kHasBtpHeaderParam, has_btp_header_)) {
#endif
    ROS_LOG(WARN, "Parameter '%s' is not set, defaulting to '%s'", kHasBtpHeaderParam.c_str(), kHasBtpHeaderParamDefault ? "true" : "false");
  }

  // load etsi_types
#ifdef ROS1
  if (!private_node_handle_.param<std::string>(kEtsiTypesParam, etsi_types_, kEtsiTypesParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss;
  ss << "list of ETSI types to convert, one of ";
  for (const auto& e : kEtsiTypesParamDefault) ss << e << ", ";
  param_desc.description = ss.str();
  this->declare_parameter(kEtsiTypesParam, kEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kEtsiTypesParam, etsi_types_)) {
#endif
    ROS_LOG(WARN, "Parameter '%s' is not set, defaulting to all", kEtsiTypesParam.c_str());
  }

  // check etsi_types
  for (const auto& e : etsi_types_) {
    if (std::find(kEtsiTypesParamDefault.begin(), kEtsiTypesParamDefault.end(), e) == kEtsiTypesParamDefault.end())
      ROS_LOG(WARN, "Invalid value '%s' for parameter '%s', skipping", e.c_str(), kEtsiTypesParam.c_str());
  }
  if (!has_btp_header_ && etsi_types_.size() > 1) {
    ROS_LOG(WARN, "Parameter '%s' is set to 'false', but multiple 'etsi_types' are configured, dropping all but the first", kHasBtpHeaderParam.c_str());
    etsi_types_.resize(1);
  }
}


void Converter::setup() {

  // create subscribers and publishers
#ifdef ROS1
  publisher_udp_ = std::make_shared<ros::Publisher>(private_node_handle_.advertise<UdpPacket>(kOutputTopicUdp, 1));
  publishers_["cam"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<cam_msgs::CAM>(kOutputTopicCam, 1));
  publishers_["denm"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<denm_msgs::DENM>(kOutputTopicDenm, 1));
  publishers_["spatem"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<spatem_msgs::SPATEM>(kOutputTopicSpatem, 1));
  publishers_["mapem"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<mapem_msgs::MAPEM>(kOutputTopicMapem, 1));
  subscriber_udp_ = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicUdp, 1, &Converter::udpCallback, this));
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "cam") != etsi_types_.end()) {
    subscribers_["cam"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicCam, 1, &Converter::rosCallbackCam, this));
    ROS_LOG(INFO, "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["cam"]->getTopic().c_str());
    ROS_LOG(INFO, "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "denm") != etsi_types_.end()) {
    subscribers_["denm"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicDenm, 1, &Converter::rosCallbackDenm, this));
    ROS_LOG(INFO, "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["denm"]->getTopic().c_str());
    ROS_LOG(INFO, "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "spatem") != etsi_types_.end()) {
    subscribers_["spatem"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicSpatem, 1, &Converter::rosCallbackSpatem, this));
    ROS_LOG(INFO, "Converting UDP messages of type SPATEM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["spatem"]->getTopic().c_str());
    ROS_LOG(INFO, "Converting native ROS SPATEMs on '%s' to UDP messages on '%s'", subscribers_["spatem"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "mapem") != etsi_types_.end()) {
    subscribers_["mapem"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicMapem, 1, &Converter::rosCallbackMapem, this));
    ROS_LOG(INFO, "Converting UDP messages of type MAPEM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["mapem"]->getTopic().c_str());
    ROS_LOG(INFO, "Converting native ROS MAPEMs on '%s' to UDP messages on '%s'", subscribers_["mapem"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
#else
  publisher_udp_ = this->create_publisher<UdpPacket>(kOutputTopicUdp, 1);
  publisher_cam_ = this->create_publisher<cam_msgs::CAM>(kOutputTopicCam, 1);
  publisher_denm_ = this->create_publisher<denm_msgs::DENM>(kOutputTopicDenm, 1);
  publisher_spatem_ = this->create_publisher<spatem_msgs::SPATEM>(kOutputTopicSpatem, 1);
  publisher_mapem_ = this->create_publisher<mapem_msgs::MAPEM>(kOutputTopicMapem, 1);
  subscriber_udp_ = this->create_subscription<UdpPacket>(kInputTopicUdp, 1, std::bind(&Converter::udpCallback, this, std::placeholders::_1));
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "cam") != etsi_types_.end()) {
    std::function<void(const cam_msgs::CAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cam_msgs::CAM, CAM_t>, this, std::placeholders::_1, "CAM", &asn_DEF_CAM, std::function<void(const cam_msgs::CAM&, CAM_t&)>(etsi_its_cam_conversion::toStruct_CAM));
    subscribers_["cam"] = this->create_subscription<cam_msgs::CAM>(kInputTopicCam, 1, callback);
    ROS_LOG(INFO, "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cam_->get_topic_name());
    ROS_LOG(INFO, "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "denm") != etsi_types_.end()) {
    std::function<void(const denm_msgs::DENM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<denm_msgs::DENM, DENM_t>, this, std::placeholders::_1, "DENM", &asn_DEF_DENM, std::function<void(const denm_msgs::DENM&, DENM_t&)>(etsi_its_denm_conversion::toStruct_DENM));
    subscribers_["denm"] = this->create_subscription<denm_msgs::DENM>(kInputTopicDenm, 1, callback);
    ROS_LOG(INFO, "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_denm_->get_topic_name());
    ROS_LOG(INFO, "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "spatem") != etsi_types_.end()) {
    std::function<void(const spatem_msgs::SPATEM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<spatem_msgs::SPATEM, SPATEM_t>, this, std::placeholders::_1, "SPATEM", &asn_DEF_SPATEM, std::function<void(const spatem_msgs::SPATEM&, SPATEM_t&)>(etsi_its_spatem_conversion::toStruct_SPATEM));
    subscribers_["spatem"] = this->create_subscription<spatem_msgs::SPATEM>(kInputTopicSpatem, 1, callback);
    ROS_LOG(INFO, "Converting UDP messages of type SPATEM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_spatem_->get_topic_name());
    ROS_LOG(INFO, "Converting native ROS SPATEMs on '%s' to UDP messages on '%s'", subscribers_["spatem"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(etsi_types_.begin(), etsi_types_.end(), "mapem") != etsi_types_.end()) {
    std::function<void(const mapem_msgs::MAPEM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<mapem_msgs::MAPEM, MAPEM_t>, this, std::placeholders::_1, "MAPEM", &asn_DEF_MAPEM, std::function<void(const mapem_msgs::MAPEM&, MAPEM_t&)>(etsi_its_mapem_conversion::toStruct_MAPEM));
    subscribers_["mapem"] = this->create_subscription<mapem_msgs::MAPEM>(kInputTopicMapem, 1, callback);
    ROS_LOG(INFO, "Converting UDP messages of type MAPEM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_mapem_->get_topic_name());
    ROS_LOG(INFO, "Converting native ROS MAPEMs on '%s' to UDP messages on '%s'", subscribers_["mapem"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
#endif
}


template <typename T_struct>
bool Converter::decodeBufferToStruct(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, T_struct* asn1_struct) {

  asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, (void **)&asn1_struct, buffer, size);
  if (ret.code != RC_OK) {
    ROS_LOG(ERROR, "Failed to decode message");
    return false;
  }
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_CAM, asn1_struct);

  return true;
}


template <typename T_ros, typename T_struct>
T_ros Converter::structToRosMessage(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn) {

  T_ros msg;
  conversion_fn(asn1_struct, msg);

  return msg;
}


template <typename T_ros, typename T_struct>
bool Converter::decodeBufferToRosMessage(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_struct&, T_ros&)> conversion_fn, T_ros& msg) {

  T_struct* asn1_struct = nullptr;
  bool success = this->decodeBufferToStruct(buffer, size, type_descriptor, asn1_struct);
  if (!success) return false;

  msg = this->structToRosMessage(*asn1_struct, type_descriptor, conversion_fn);

  return true;
}


template <typename T_ros, typename T_struct>
T_struct Converter::rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) {

  T_struct asn1_struct;
  conversion_fn(msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, type_descriptor, &asn1_struct);

  return asn1_struct;
}


template <typename T_struct>
bool Converter::encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t* buffer, int& size) {

  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(type_descriptor, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
    ROS_LOG(ERROR, "Check of struct failed: %s", error_buffer);
    return false;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, &asn1_struct);
  if (ret.result.encoded == -1) {
    ROS_LOG(ERROR, "Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return false;
  }

  buffer = static_cast<uint8_t*>(ret.buffer);
  size = ret.result.encoded;

  return true;
}


UdpPacket Converter::bufferToUdpPacketMessage(const uint8_t* buffer, const int size) {

#ifdef ROS1
  UdpPacket udp_msg;
#else
  UdpPacket udp_msg;
#endif
  if (has_btp_header_) {
    // add BTP-Header if enabled
    uint16_t destination_port = htons(kBtpHeaderDestinationPortCam);
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
bool Converter::encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) {

  // convert ROS msg to struct
  auto asn1_struct = this->rosMessageToStruct(msg, type_descriptor, conversion_fn);

  // encode struct to ASN1 bitstring
  uint8_t* buffer;
  int buffer_size;
  bool successful = this->encodeStructToBuffer(asn1_struct, type_descriptor, buffer, buffer_size);
  if (!successful) return false;

  // copy bitstring to ROS UDP msg
  udp_msg = this->bufferToUdpPacketMessage(buffer, buffer_size);

  return true;
}


#ifdef ROS1
void Converter::udpCallback(const UdpPacket::ConstPtr udp_msg) {
#else
void Converter::udpCallback(const UdpPacket::UniquePtr udp_msg) {
#endif

  ROS_LOG(DEBUG, "Received bitstring");

  // decode BTP-Header if enabled
  std::string detected_etsi_type = etsi_types_[0];
  int offset = 0;
  if (has_btp_header_) {
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

    // decode buffer to ROS msg
    cam_msgs::CAM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[offset], udp_msg->data.size() - offset, &asn_DEF_CAM, std::function<void(const CAM_t&, cam_msgs::CAM&)>(etsi_its_cam_conversion::toRos_CAM), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["cam"]->publish(msg);
#else
    publisher_cam_->publish(msg);
#endif
    ROS_LOG(DEBUG, "Published CAM");

  } else if (detected_etsi_type == "denm") {

    // decode buffer to ROS msg
    denm_msgs::DENM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[offset], udp_msg->data.size() - offset, &asn_DEF_DENM, std::function<void(const DENM_t&, denm_msgs::DENM&)>(etsi_its_denm_conversion::toRos_DENM), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["denm"]->publish(msg);
#else
    publisher_denm_->publish(msg);
#endif
    ROS_LOG(DEBUG, "Published DENM");

  } else if (detected_etsi_type == "spatem") {

    // decode buffer to ROS msg
    spatem_msgs::SPATEM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[offset], udp_msg->data.size() - offset, &asn_DEF_SPATEM, std::function<void(const SPATEM_t&, spatem_msgs::SPATEM&)>(etsi_its_spatem_conversion::toRos_SPATEM), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["spatem"]->publish(msg);
#else
    publisher_spatem_->publish(msg);
#endif
    ROS_LOG(DEBUG, "Published SPATEM");

  } else if (detected_etsi_type == "mapem") {

    // decode buffer to ROS msg
    mapem_msgs::MAPEM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[offset], udp_msg->data.size() - offset, &asn_DEF_MAPEM, std::function<void(const MAPEM_t&, mapem_msgs::MAPEM&)>(etsi_its_mapem_conversion::toRos_MAPEM), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["mapem"]->publish(msg);
#else
    publisher_mapem_->publish(msg);
#endif
    ROS_LOG(DEBUG, "Published MAPEM");

  } else {
    ROS_LOG(ERROR, "Detected ETSI message type '%s' not yet supported, dropping message", detected_etsi_type.c_str());
    return;
  }
}


template <typename T_ros, typename T_struct>
#ifdef ROS1
void Converter::rosCallback(const T_ros::ConstPtr msg,
#else
void Converter::rosCallback(const typename T_ros::UniquePtr msg,
#endif
                            const std::string& type, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) {

  ROS_LOG(DEBUG, "Received %s", type.c_str());

  // encode ROS msg to UDP msg
  UdpPacket udp_msg;
  bool success = this->encodeRosMessageToUdpPacketMessage<T_ros, T_struct>(*msg, udp_msg, type_descriptor, conversion_fn);
  if (!success) return;

  // publish UDP msg
  publisher_udp_->publish(udp_msg);
  ROS_LOG(DEBUG, "Published %s bitstring", type.c_str());
}


}  // end of namespace
