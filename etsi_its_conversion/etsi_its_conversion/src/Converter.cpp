#include <algorithm>

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include "etsi_its_conversion/Converter.h"


PLUGINLIB_EXPORT_CLASS(etsi_its_conversion::Converter, nodelet::Nodelet)


namespace etsi_its_conversion {


const std::string Converter::kInputTopicUdp{"udp/in"};
const std::string Converter::kOutputTopicCam{"cam/out"};
const std::string Converter::kInputTopicCam{"cam/in"};
const std::string Converter::kOutputTopicUdp{"udp/out"};


bool logLevelIsDebug() {

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
  return false;
}


void Converter::onInit() {

  private_node_handle_ = this->getMTPrivateNodeHandle();

  this->loadParameters();
  this->setup();
}


void Converter::loadParameters() {

  std::vector<std::string> known_etsi_types = {"auto", "cam"};
  if (!private_node_handle_.param<std::string>("etsi_type", etsi_type_, "auto")) {
    NODELET_WARN("Parameter '%s' is not set, defaulting to '%s'", "etsi_type", "auto");
  }
  if (std::find(known_etsi_types.begin(), known_etsi_types.end(), etsi_type_) == known_etsi_types.end()) {
    NODELET_WARN("Invalid value for parameter '%s', defaulting to '%s'", "etsi_type", "auto");
  }
}


void Converter::setup() {

  // create subscribers and publishers
  publisher_udp_ = private_node_handle_.advertise<udp_msgs::UdpPacket>(kOutputTopicUdp, 1);
  publishers_["cam"] = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>(kOutputTopicCam, 1);
  subscriber_udp_ = private_node_handle_.subscribe(kInputTopicUdp, 1, &Converter::udpCallback, this);
  subscribers_["cam"] = private_node_handle_.subscribe(kInputTopicCam, 1, &Converter::rosCallbackCam, this);
  
  NODELET_INFO("Converting UDP messages of type '%s' on '%s' to native ROS messages on '%s'", etsi_type_.c_str(), subscriber_udp_.getTopic().c_str(), publishers_["cam"].getTopic().c_str());
  NODELET_INFO("Converting native ROS CAM messages on '%s' to UDP messages on '%s'", subscribers_["cam"].getTopic().c_str(), publisher_udp_.getTopic().c_str());
}


void Converter::udpCallback(const udp_msgs::UdpPacket::ConstPtr udp_msg) {

  NODELET_DEBUG("Received CAM bitstring");

  // TODO: decode and strip TP-Header, if etsi_type==auto

  // decode ASN1 bitstring to struct
  CAM_t* asn1_struct = nullptr;
  asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&asn1_struct, &udp_msg->data[0], udp_msg->data.size());
  if (ret.code != RC_OK) {
    NODELET_ERROR("Failed to decode message");
    return;
  }
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_CAM, asn1_struct);

  // convert struct to ROS msg
  etsi_its_cam_msgs::CAM msg;
  etsi_its_cam_conversion::toRos_CAM(*asn1_struct, msg);

  publishers_["cam"].publish(msg);
  NODELET_DEBUG("Published CAM");
}


void Converter::rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg) {

  NODELET_DEBUG("Received CAM");

  // convert ROS msg to struct
  CAM_t asn1_struct;
  etsi_its_cam_conversion::toStruct_CAM(*msg, asn1_struct);
  if (logLevelIsDebug()) asn_fprint(stdout, &asn_DEF_CAM, &asn1_struct);

  // encode struct to ASN1 bitstring
  char error_buffer[1024];
  size_t error_length = sizeof(error_buffer);
  int check_ret = asn_check_constraints(&asn_DEF_CAM, &asn1_struct, error_buffer, &error_length);
  if (check_ret != 0) {
    NODELET_ERROR("Check of struct failed: %s", error_buffer);
    return;
  }
  asn_encode_to_new_buffer_result_t ret = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &asn1_struct);
  if (ret.result.encoded == -1) {
    NODELET_ERROR("Failed to encode message: %s", ret.result.failed_type->xml_tag);
    return;
  }

  // TODO: add BTP-header, if etsi_type==auto

  udp_msgs::UdpPacket udp_msg;
  udp_msg.data = std::vector<uint8_t>((uint8_t*)ret.buffer, (uint8_t*)ret.buffer + (int)ret.result.encoded);
  publisher_udp_.publish(udp_msg);
  NODELET_DEBUG("Published CAM bitstring");
}


}  // end of namespace
