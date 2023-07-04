#include <pluginlib/class_list_macros.h>

#include "etsi_its_conversion/Converter.h"


PLUGINLIB_EXPORT_CLASS(etsi_its_conversion::Converter, nodelet::Nodelet)


namespace etsi_its_conversion {


const std::string Converter::kInputTopicCam{"in/cam"};
const std::string Converter::kOutputTopicCam{"out/cam"};
const std::string Converter::kInputTopicAsn1Cam{"bitstring/in/cam"};
const std::string Converter::kOutputTopicAsn1Cam{"bitstring/out/cam"};


void Converter::onInit() {

  private_node_handle_ = this->getMTPrivateNodeHandle();

  // create subscribers and publishers
  publishers_["cam"] = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>(kOutputTopicCam, 1);
  publishers_asn1_["cam"] = private_node_handle_.advertise<bitstring_msgs::UInt8ArrayStamped>(kOutputTopicAsn1Cam, 1);
  subscribers_["cam"] = private_node_handle_.subscribe(kInputTopicCam, 1, &Converter::rosCallbackCam, this);
  subscribers_asn1_["cam"] = private_node_handle_.subscribe(kInputTopicAsn1Cam, 1, &Converter::asn1CallbackCam, this);
  NODELET_INFO("Converting CAM bitstrings on '%s' to CAM messages on '%s'", subscribers_asn1_["cam"].getTopic().c_str(), publishers_["cam"].getTopic().c_str());
  NODELET_INFO("Converting CAM messages on '%s' to CAM bitstrings on '%s'", subscribers_["cam"].getTopic().c_str(), publishers_asn1_["cam"].getTopic().c_str());
}


void Converter::asn1CallbackCam(const bitstring_msgs::UInt8ArrayStamped::ConstPtr bitstring_msg) {

  NODELET_DEBUG("Received CAM bitstring");

  // decode ASN1 bitstring to struct
  CAM_t* asn1_struct;
  asn_dec_rval_t ret = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&asn1_struct, &bitstring_msg->data[0], bitstring_msg->data.size());
  if (ret.code != RC_OK) {
    NODELET_ERROR("Failed to decode message");
    return;
  }
  // asn_fprint(stdout, &asn_DEF_CAM, asn1_struct);

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
  // asn_fprint(stdout, &asn_DEF_CAM, &asn1_struct);

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

  bitstring_msgs::UInt8ArrayStamped bitstring_msg;
  bitstring_msg.header.stamp = ros::Time::now(); // TODO: use msg time?
  bitstring_msg.header.frame_id = ""; // TODO
  bitstring_msg.data = std::vector<uint8_t>((uint8_t*)ret.buffer, (uint8_t*)ret.buffer + (int)ret.result.encoded);
  publishers_asn1_["cam"].publish(bitstring_msg);
  NODELET_DEBUG("Published CAM bitstring");
}


}  // end of namespace
