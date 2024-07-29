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
#define ROS12_LOG(level, ...) NODELET_##level(__VA_ARGS__)
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(etsi_its_conversion::Converter, nodelet::Nodelet)
#else
#define ROS12_LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(etsi_its_conversion::Converter)
#endif

namespace etsi_its_conversion {

const int kBtpHeaderDestinationPortCam{2001};
const int kBtpHeaderDestinationPortDenm{2002};
const int kBtpHeaderDestinationPortMapem{2003};
const int kBtpHeaderDestinationPortSpatem{2004};
const int kBtpHeaderDestinationPortIvi{2006};
const int kBtpHeaderDestinationPortCpmTs{2009};

#ifdef ROS1
const std::string Converter::kInputTopicUdp{"udp/in"};
const std::string Converter::kOutputTopicUdp{"udp/out"};
const std::string Converter::kInputTopicCam{"cam/in"};
const std::string Converter::kOutputTopicCam{"cam/out"};
const std::string Converter::kInputTopicCamTs{"cam_ts/in"};
const std::string Converter::kOutputTopicCamTs{"cam_ts/out"};
const std::string Converter::kInputTopicCpmTs{"cpm_ts/in"};
const std::string Converter::kOutputTopicCpmTs{"cpm_ts/out"};
const std::string Converter::kInputTopicDenm{"denm/in"};
const std::string Converter::kOutputTopicDenm{"denm/out"};
#else
const std::string Converter::kInputTopicUdp{"~/udp/in"};
const std::string Converter::kOutputTopicUdp{"~/udp/out"};
const std::string Converter::kInputTopicCam{"~/cam/in"};
const std::string Converter::kOutputTopicCam{"~/cam/out"};
const std::string Converter::kInputTopicCamTs{"~/cam_ts/in"};
const std::string Converter::kOutputTopicCamTs{"~/cam_ts/out"};
const std::string Converter::kInputTopicCpmTs{"~/cpm_ts/in"};
const std::string Converter::kOutputTopicCpmTs{"~/cpm_ts/out"};
const std::string Converter::kInputTopicDenm{"~/denm/in"};
const std::string Converter::kOutputTopicDenm{"~/denm/out"};
#endif

const std::string Converter::kHasBtpDestinationPortParam{"has_btp_destination_port"};
const bool Converter::kHasBtpDestinationPortParamDefault{true};
const std::string Converter::kBtpDestinationPortOffsetParam{"btp_destination_port_offset"};
const int Converter::kBtpDestinationPortOffsetParamDefault{8};
const std::string Converter::kEtsiMessagePayloadOffsetParam{"etsi_message_payload_offset"};
const int Converter::kEtsiMessagePayloadOffsetParamDefault{78};
const std::string Converter::kRos2UdpEtsiTypesParam{"ros2udp_etsi_types"};
const std::string Converter::kUdp2RosEtsiTypesParam{"udp2ros_etsi_types"};
const std::vector<std::string> Converter::kRos2UdpEtsiTypesParamDefault{"cam", "cam_ts", "cpm_ts", "denm"};
const std::vector<std::string> Converter::kUdp2RosEtsiTypesParamDefault{"cam", "cpm_ts", "denm"};
const std::string Converter::kSubscriberQueueSizeParam{"subscriber_queue_size"};
const int Converter::kSubscriberQueueSizeParamDefault{10};
const std::string Converter::kPublisherQueueSizeParam{"publisher_queue_size"};
const int Converter::kPublisherQueueSizeParamDefault{10};


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
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "whether incoming/outgoing UDP messages include a 4-byte BTP header";
  this->declare_parameter(kHasBtpDestinationPortParam, kHasBtpDestinationPortParamDefault, param_desc);
  if (!this->get_parameter(kHasBtpDestinationPortParam, has_btp_destination_port_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to '%s'", kHasBtpDestinationPortParam.c_str(), kHasBtpDestinationPortParamDefault ? "true" : "false");
  }

  // load btp_destination_port_offset
#ifdef ROS1
  if (!private_node_handle_.param<int>(kBtpDestinationPortOffsetParam, btp_destination_port_offset_, kBtpDestinationPortOffsetParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kBtpDestinationPortOffsetParam, kBtpDestinationPortOffsetParamDefault, param_desc);
  if (!this->get_parameter(kBtpDestinationPortOffsetParam, btp_destination_port_offset_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to '%d'", kBtpDestinationPortOffsetParam.c_str(), kBtpDestinationPortOffsetParamDefault);
  }

  // load etsi_message_payload_offset
#ifdef ROS1
  if (!private_node_handle_.param<int>(kEtsiMessagePayloadOffsetParam, etsi_message_payload_offset_, kEtsiMessagePayloadOffsetParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "number of bytes until actual message payload starts in incoming UDP message (optionally including BTP header)";
  this->declare_parameter(kEtsiMessagePayloadOffsetParam, kEtsiMessagePayloadOffsetParamDefault, param_desc);
  if (!this->get_parameter(kEtsiMessagePayloadOffsetParam, etsi_message_payload_offset_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to '%d'", kEtsiMessagePayloadOffsetParam.c_str(), kEtsiMessagePayloadOffsetParamDefault);
  }

  // load ros2udp_etsi_types_
#ifdef ROS1
  if (!private_node_handle_.param<std::vector<std::string>>(kRos2UdpEtsiTypesParam, ros2udp_etsi_types_, kRos2UdpEtsiTypesParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss_ros2udp;
  ss_ros2udp << "list of ETSI types to convert from ROS to UDP, one of ";
  for (const auto& e : kRos2UdpEtsiTypesParamDefault) ss_ros2udp << e << ", ";
  param_desc.description = ss_ros2udp.str();
  this->declare_parameter(kRos2UdpEtsiTypesParam, kRos2UdpEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kRos2UdpEtsiTypesParam, ros2udp_etsi_types_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to all", kRos2UdpEtsiTypesParam.c_str());
  }

  // check ros2udp_etsi_types
  for (const auto& e : ros2udp_etsi_types_) {
    if (std::find(kRos2UdpEtsiTypesParamDefault.begin(), kRos2UdpEtsiTypesParamDefault.end(), e) == kRos2UdpEtsiTypesParamDefault.end())
      ROS12_LOG(WARN, "Invalid value '%s' for parameter '%s', skipping", e.c_str(), kRos2UdpEtsiTypesParam.c_str());
  }

  // load udp2ros_etsi_types_
#ifdef ROS1
  if (!private_node_handle_.param<std::vector<std::string>>(kUdp2RosEtsiTypesParam, udp2ros_etsi_types_, kUdp2RosEtsiTypesParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  std::stringstream ss_udp2ros;
  ss_udp2ros << "list of ETSI types to convert from UDP to ROS, one of ";
  for (const auto& e : kUdp2RosEtsiTypesParamDefault) ss_udp2ros << e << ", ";
  param_desc.description = ss_udp2ros.str();
  this->declare_parameter(kUdp2RosEtsiTypesParam, kUdp2RosEtsiTypesParamDefault, param_desc);
  if (!this->get_parameter(kUdp2RosEtsiTypesParam, udp2ros_etsi_types_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to all", kUdp2RosEtsiTypesParam.c_str());
  }

  // check udp2ros_etsi_types
  for (const auto& e : udp2ros_etsi_types_) {
    if (std::find(kUdp2RosEtsiTypesParamDefault.begin(), kUdp2RosEtsiTypesParamDefault.end(), e) == kUdp2RosEtsiTypesParamDefault.end())
      ROS12_LOG(WARN, "Invalid value '%s' for parameter '%s', skipping", e.c_str(), kUdp2RosEtsiTypesParam.c_str());
  }
  if (!has_btp_destination_port_ && udp2ros_etsi_types_.size() > 1) {
    ROS12_LOG(WARN, "Parameter '%s' is set to 'false', but multiple 'udp2ros_etsi_types' are configured, dropping all but the first", kHasBtpDestinationPortParam.c_str());
    udp2ros_etsi_types_.resize(1);
  }

  // load subscriber_queue_size
#ifdef ROS1
  if (!private_node_handle_.param<int>(kSubscriberQueueSizeParam, subscriber_queue_size_, kSubscriberQueueSizeParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "queue size for incoming ROS messages";
  this->declare_parameter(kSubscriberQueueSizeParam, kSubscriberQueueSizeParamDefault, param_desc);
  if (!this->get_parameter(kSubscriberQueueSizeParam, subscriber_queue_size_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to '%d'", kSubscriberQueueSizeParam.c_str(), kSubscriberQueueSizeParamDefault);
  }

  // load publisher_queue_size
#ifdef ROS1
  if (!private_node_handle_.param<int>(kPublisherQueueSizeParam, publisher_queue_size_, kPublisherQueueSizeParamDefault)) {
#else
  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "queue size for outgoing ROS messages";
  this->declare_parameter(kPublisherQueueSizeParam, kPublisherQueueSizeParamDefault, param_desc);
  if (!this->get_parameter(kPublisherQueueSizeParam, publisher_queue_size_)) {
#endif
    ROS12_LOG(WARN, "Parameter '%s' is not set, defaulting to '%d'", kPublisherQueueSizeParam.c_str(), kPublisherQueueSizeParamDefault);
  }
}


void Converter::setup() {

  // create subscribers and publishers
#ifdef ROS1
  publisher_udp_ = std::make_shared<ros::Publisher>(private_node_handle_.advertise<UdpPacket>(kOutputTopicUdp, publisher_queue_size_));
  subscriber_udp_ = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe(kInputTopicUdp, subscriber_queue_size_, &Converter::udpCallback, this));
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam") != udp2ros_etsi_types_.end()) {
    publishers_["cam"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<cam_msgs::CAM>(kOutputTopicCam, publisher_queue_size_));
    ROS12_LOG(INFO, "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["cam"]->getTopic().c_str());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam") != ros2udp_etsi_types_.end()) {
    boost::function<void(const cam_msgs::CAM::ConstPtr)> callback =
      boost::bind(&Converter::rosCallback<cam_msgs::CAM, cam_CAM_t>, this, _1, "cam", &asn_DEF_cam_CAM, std::function<void(const cam_msgs::CAM&, cam_CAM_t&)>(etsi_its_cam_conversion::toStruct_CAM));
    subscribers_["cam"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe<cam_msgs::CAM>(kInputTopicCam, subscriber_queue_size_, callback));
    ROS12_LOG(INFO, "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam_ts") != udp2ros_etsi_types_.end()) {
    publishers_["cam_ts"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<cam_ts_msgs::CAM>(kOutputTopicCamTs, publisher_queue_size_));
    ROS12_LOG(INFO, "Converting UDP messages of type CAM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["cam_ts"]->getTopic().c_str());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam_ts") != ros2udp_etsi_types_.end()) {
    boost::function<void(const cam_ts_msgs::CAM::ConstPtr)> callback =
      boost::bind(&Converter::rosCallback<cam_ts_msgs::CAM, cam_ts_CAM_t>, this, _1, "cam_ts", &asn_DEF_cam_ts_CAM, std::function<void(const cam_ts_msgs::CAM&, cam_ts_CAM_t&)>(etsi_its_cam_ts_conversion::toStruct_CAM));
    subscribers_["cam_ts"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe<cam_ts_msgs::CAM>(kInputTopicCamTs, subscriber_queue_size_, callback));
    ROS12_LOG(INFO, "Converting native ROS CAM (TS) on '%s' to UDP messages on '%s'", subscribers_["cam_ts"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cpm_ts") != udp2ros_etsi_types_.end()) {
    publishers_["cpm_ts"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<cpm_ts_msgs::CollectivePerceptionMessage>(kOutputTopicCpmTs, publisher_queue_size_));
    ROS12_LOG(INFO, "Converting UDP messages of type CPM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["cpm_ts"]->getTopic().c_str());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cpm_ts") != ros2udp_etsi_types_.end()) {
    boost::function<void(const cpm_ts_msgs::CollectivePerceptionMessage::ConstPtr)> callback =
      boost::bind(&Converter::rosCallback<cpm_ts_msgs::CollectivePerceptionMessage, cpm_ts_CollectivePerceptionMessage_t>, this, _1, "cpm_ts", &asn_DEF_cpm_ts_CollectivePerceptionMessage, std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage&, cpm_ts_CollectivePerceptionMessage_t&)>(etsi_its_cpm_ts_conversion::toStruct_CollectivePerceptionMessage));
    subscribers_["cpm_ts"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe<cpm_ts_msgs::CollectivePerceptionMessage>(kInputTopicCpmTs, subscriber_queue_size_, callback));
    ROS12_LOG(INFO, "Converting native ROS CPM (TS) on '%s' to UDP messages on '%s'", subscribers_["cpm_ts"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm") != udp2ros_etsi_types_.end()) {
    publishers_["denm"] = std::make_shared<ros::Publisher>(private_node_handle_.advertise<denm_msgs::DENM>(kOutputTopicDenm, publisher_queue_size_));
    ROS12_LOG(INFO, "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->getTopic().c_str(), publishers_["denm"]->getTopic().c_str());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "denm") != ros2udp_etsi_types_.end()) {
    boost::function<void(const denm_msgs::DENM::ConstPtr)> callback =
      boost::bind(&Converter::rosCallback<denm_msgs::DENM, denm_DENM_t>, this, _1, "denm", &asn_DEF_denm_DENM, std::function<void(const denm_msgs::DENM&, denm_DENM_t&)>(etsi_its_denm_conversion::toStruct_DENM));
    subscribers_["denm"] = std::make_shared<ros::Subscriber>(private_node_handle_.subscribe<denm_msgs::DENM>(kInputTopicDenm, subscriber_queue_size_, callback));
    ROS12_LOG(INFO, "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"]->getTopic().c_str(), publisher_udp_->getTopic().c_str());
  }
#else
  publisher_udp_ = this->create_publisher<UdpPacket>(kOutputTopicUdp, publisher_queue_size_);
  subscriber_udp_ = this->create_subscription<UdpPacket>(kInputTopicUdp, subscriber_queue_size_, std::bind(&Converter::udpCallback, this, std::placeholders::_1));
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam") != udp2ros_etsi_types_.end()) {
    publisher_cam_ = this->create_publisher<cam_msgs::CAM>(kOutputTopicCam, publisher_queue_size_);
    ROS12_LOG(INFO, "Converting UDP messages of type CAM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cam_->get_topic_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam") != ros2udp_etsi_types_.end()) {
    std::function<void(const cam_msgs::CAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cam_msgs::CAM, cam_CAM_t>, this, std::placeholders::_1, "cam", &asn_DEF_cam_CAM, std::function<void(const cam_msgs::CAM&, cam_CAM_t&)>(etsi_its_cam_conversion::toStruct_CAM));
    subscribers_["cam"] = this->create_subscription<cam_msgs::CAM>(kInputTopicCam, subscriber_queue_size_, callback);
    ROS12_LOG(INFO, "Converting native ROS CAMs on '%s' to UDP messages on '%s'", subscribers_["cam"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam_ts") != udp2ros_etsi_types_.end()) {
    publisher_cam_ts_ = this->create_publisher<cam_ts_msgs::CAM>(kOutputTopicCamTs, publisher_queue_size_);
    ROS12_LOG(INFO, "Converting UDP messages of type CAM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cam_ts_->get_topic_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cam_ts") != ros2udp_etsi_types_.end()) {
    std::function<void(const cam_ts_msgs::CAM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cam_ts_msgs::CAM, cam_ts_CAM_t>, this, std::placeholders::_1, "cam_ts", &asn_DEF_cam_ts_CAM, std::function<void(const cam_ts_msgs::CAM&, cam_ts_CAM_t&)>(etsi_its_cam_ts_conversion::toStruct_CAM));
    subscribers_["cam_ts"] = this->create_subscription<cam_ts_msgs::CAM>(kInputTopicCamTs, subscriber_queue_size_, callback);
    ROS12_LOG(INFO, "Converting native ROS CAM (TS) on '%s' to UDP messages on '%s'", subscribers_["cam_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cpm_ts") != udp2ros_etsi_types_.end()) {
    publisher_cpm_ts_ = this->create_publisher<cpm_ts_msgs::CollectivePerceptionMessage>(kOutputTopicCpmTs, publisher_queue_size_);
    ROS12_LOG(INFO, "Converting UDP messages of type CPM (TS) on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_cpm_ts_->get_topic_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "cpm_ts") != ros2udp_etsi_types_.end()) {
    std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<cpm_ts_msgs::CollectivePerceptionMessage, cpm_ts_CollectivePerceptionMessage_t>, this, std::placeholders::_1, "cpm_ts", &asn_DEF_cpm_ts_CollectivePerceptionMessage, std::function<void(const cpm_ts_msgs::CollectivePerceptionMessage&, cpm_ts_CollectivePerceptionMessage_t&)>(etsi_its_cpm_ts_conversion::toStruct_CollectivePerceptionMessage));
    subscribers_["cpm_ts"] = this->create_subscription<cpm_ts_msgs::CollectivePerceptionMessage>(kInputTopicCpmTs, subscriber_queue_size_, callback);
    ROS12_LOG(INFO, "Converting native ROS CPMs (TS) on '%s' to UDP messages on '%s'", subscribers_["cpm_ts"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
  if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "denm") != udp2ros_etsi_types_.end()) {
    publisher_denm_ = this->create_publisher<denm_msgs::DENM>(kOutputTopicDenm, publisher_queue_size_);
    ROS12_LOG(INFO, "Converting UDP messages of type DENM on '%s' to native ROS messages on '%s'", subscriber_udp_->get_topic_name(), publisher_denm_->get_topic_name());
  }
  if (std::find(ros2udp_etsi_types_.begin(), ros2udp_etsi_types_.end(), "denm") != ros2udp_etsi_types_.end()) {
    std::function<void(const denm_msgs::DENM::UniquePtr)> callback =
      std::bind(&Converter::rosCallback<denm_msgs::DENM, denm_DENM_t>, this, std::placeholders::_1, "denm", &asn_DEF_denm_DENM, std::function<void(const denm_msgs::DENM&, denm_DENM_t&)>(etsi_its_denm_conversion::toStruct_DENM));
    subscribers_["denm"] = this->create_subscription<denm_msgs::DENM>(kInputTopicDenm, subscriber_queue_size_, callback);
    ROS12_LOG(INFO, "Converting native ROS DENMs on '%s' to UDP messages on '%s'", subscribers_["denm"]->get_topic_name(), publisher_udp_->get_topic_name());
  }
#endif
}


template <typename T_struct>
bool Converter::decodeBufferToStruct(const uint8_t* buffer, const int size, const asn_TYPE_descriptor_t* type_descriptor, T_struct* asn1_struct) {

  asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, (void **)&asn1_struct, buffer, size);
  if (ret.code != RC_OK) {
    ROS12_LOG(ERROR, "Failed to decode message");
    return false;
  }
  if (logLevelIsDebug()) asn_fprint(stdout, type_descriptor, asn1_struct);

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

  T_struct asn1_struct{};
  bool success = this->decodeBufferToStruct(buffer, size, type_descriptor, &asn1_struct);
  if (success) msg = this->structToRosMessage(asn1_struct, type_descriptor, conversion_fn);
  ASN_STRUCT_FREE_CONTENTS_ONLY(*type_descriptor, &asn1_struct);

  return success;
}


template <typename T_ros, typename T_struct>
T_struct Converter::rosMessageToStruct(const T_ros& msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) {

  T_struct asn1_struct{};
  conversion_fn(msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, type_descriptor, &asn1_struct);

  return asn1_struct;
}


template <typename T_struct>
bool Converter::encodeStructToBuffer(const T_struct& asn1_struct, const asn_TYPE_descriptor_t* type_descriptor, uint8_t*& buffer, int& size) {

  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(type_descriptor, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
    ROS12_LOG(ERROR, "Check of struct failed: %s", error_buffer);
    return false;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, type_descriptor, &asn1_struct);
  if (ret.result.encoded == -1) {
    ROS12_LOG(ERROR, "Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return false;
  }

  buffer = static_cast<uint8_t*>(ret.buffer);
  size = ret.result.encoded;

  return true;
}


UdpPacket Converter::bufferToUdpPacketMessage(const uint8_t* buffer, const int size, const int btp_header_destination_port) {

#ifdef ROS1
  UdpPacket udp_msg;
#else
  UdpPacket udp_msg;
#endif
  if (has_btp_destination_port_) {
    // add BTP destination port and destination port info
    uint16_t destination_port = htons(btp_header_destination_port);
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
bool Converter::encodeRosMessageToUdpPacketMessage(const T_ros& msg, UdpPacket& udp_msg, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn, const int btp_header_destination_port) {

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


#ifdef ROS1
void Converter::udpCallback(const UdpPacket::ConstPtr udp_msg) {
#else
void Converter::udpCallback(const UdpPacket::UniquePtr udp_msg) {
#endif

  ROS12_LOG(DEBUG, "Received bitstring (total payload size: %ld)", udp_msg->data.size());

  // auto-detect ETSI message type if BTP destination port is present
  std::string detected_etsi_type = udp2ros_etsi_types_[0];
  if (has_btp_destination_port_) {
    const uint16_t* btp_destination_port = reinterpret_cast<const uint16_t*>(&udp_msg->data[btp_destination_port_offset_]);
    uint16_t destination_port = ntohs(*btp_destination_port);
    if (destination_port == kBtpHeaderDestinationPortCam) detected_etsi_type = "cam";
    else if (destination_port == kBtpHeaderDestinationPortCpmTs) detected_etsi_type = "cpm_ts";
    else if (destination_port == kBtpHeaderDestinationPortDenm) detected_etsi_type = "denm";
    else if (destination_port == kBtpHeaderDestinationPortIvi) detected_etsi_type = "ivi";
    else if (destination_port == kBtpHeaderDestinationPortMapem) detected_etsi_type = "mapem";
    else if (destination_port == kBtpHeaderDestinationPortSpatem) detected_etsi_type = "spatem";
    else detected_etsi_type = "unknown";
  }

  const uint8_t* protocol_version = reinterpret_cast<const uint8_t*>(&udp_msg->data[etsi_message_payload_offset_]);

  int msg_size = udp_msg->data.size() - etsi_message_payload_offset_;
  ROS12_LOG(INFO, "Received ETSI message of type '%s' (protocolVersion: %d) as bitstring (message size: %d | total payload size: %ld)", detected_etsi_type.c_str(), *protocol_version , msg_size, udp_msg->data.size());

  if (detected_etsi_type == "cam" || detected_etsi_type == "cam_ts") {

    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam") != udp2ros_etsi_types_.end()) { // CAM EN v1.4.1
      cam_msgs::CAM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cam_CAM, std::function<void(const cam_CAM_t&, cam_msgs::CAM&)>(etsi_its_cam_conversion::toRos_CAM), msg);
      if (!success) return;
#ifdef ROS1
      publishers_["cam"]->publish(msg);
#else
      publisher_cam_->publish(msg);
#endif
    }
    if (std::find(udp2ros_etsi_types_.begin(), udp2ros_etsi_types_.end(), "cam_ts") != udp2ros_etsi_types_.end()) { // CAM TS v2.1.1
      cam_ts_msgs::CAM msg;
      bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cam_ts_CAM, std::function<void(const cam_ts_CAM_t&, cam_ts_msgs::CAM&)>(etsi_its_cam_ts_conversion::toRos_CAM), msg);
      if (!success) return;
#ifdef ROS1
      publishers_["cam_ts"]->publish(msg);
#else
      publisher_cam_ts_->publish(msg);
#endif
    }

  } else if (detected_etsi_type == "cpm_ts") {

    // decode buffer to ROS msg
    cpm_ts_msgs::CollectivePerceptionMessage msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_cpm_ts_CollectivePerceptionMessage, std::function<void(const cpm_ts_CollectivePerceptionMessage_t&, cpm_ts_msgs::CollectivePerceptionMessage&)>(etsi_its_cpm_ts_conversion::toRos_CollectivePerceptionMessage), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["cpm_ts"]->publish(msg);
#else
    publisher_cpm_ts_->publish(msg);
#endif

  } else if (detected_etsi_type == "denm") {

    // decode buffer to ROS msg
    denm_msgs::DENM msg;
    bool success = this->decodeBufferToRosMessage(&udp_msg->data[etsi_message_payload_offset_], msg_size, &asn_DEF_denm_DENM, std::function<void(const denm_DENM_t&, denm_msgs::DENM&)>(etsi_its_denm_conversion::toRos_DENM), msg);
    if (!success) return;

    // publish msg
#ifdef ROS1
    publishers_["denm"]->publish(msg);
#else
    publisher_denm_->publish(msg);
#endif

  } else {
    ROS12_LOG(ERROR, "Detected ETSI message type '%s' not yet supported, dropping message", detected_etsi_type.c_str());
    return;
  }

  ROS12_LOG(INFO, "Published ETSI message of type '%s' as ROS message", detected_etsi_type.c_str());
}


template <typename T_ros, typename T_struct>
#ifdef ROS1
void Converter::rosCallback(const typename T_ros::ConstPtr msg,
#else
void Converter::rosCallback(const typename T_ros::UniquePtr msg,
#endif
                            const std::string& type, const asn_TYPE_descriptor_t* type_descriptor, std::function<void(const T_ros&, T_struct&)> conversion_fn) {

  ROS12_LOG(INFO, "Received ETSI message of type '%s' as ROS message", type.c_str());

  int btp_header_destination_port = 0;
  if (type == "cam" || type == "cam_ts") btp_header_destination_port = kBtpHeaderDestinationPortCam;
  else if (type == "cpm_ts") btp_header_destination_port = kBtpHeaderDestinationPortCpmTs;
  else if (type == "denm") btp_header_destination_port = kBtpHeaderDestinationPortDenm;

  // encode ROS msg to UDP msg
  UdpPacket udp_msg;
  bool success = this->encodeRosMessageToUdpPacketMessage<T_ros, T_struct>(*msg, udp_msg, type_descriptor, conversion_fn, btp_header_destination_port);
  if (!success) return;

  // publish UDP msg
  publisher_udp_->publish(udp_msg);
  int msg_size = has_btp_destination_port_ ? udp_msg.data.size() - 4 : udp_msg.data.size();
  ROS12_LOG(INFO, "Published ETSI message of type '%s' as bitstring (message size: %d | total payload size: %ld)", type.c_str(), msg_size, udp_msg.data.size());
}


}  // end of namespace
