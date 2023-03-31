#include <ConversionNode.h>


namespace etsi_its_conversion {


ConversionNode::ConversionNode() : node_handle_(), private_node_handle_("~") {
  ROS_INFO("ConversionNode starting...");

  // setup publisher and subscriber
  cam_ros_pub_ = private_node_handle_.advertise<etsi_its_cam_msgs::CAM>("/out/CAM", 1);
  cam_ros_sub_ = private_node_handle_.subscribe("/in/CAM", 1, &ConversionNode::cam_callback, this);

  cam_asn1_pub_ = private_node_handle_.advertise<etsi_its_asn1_msgs::ASN1_udp>("/out/asn1/CAM", 1);
  cam_asn1_sub_ = private_node_handle_.subscribe("/in/asn1/CAM", 1, &ConversionNode::cam_asn1_callback, this);

  ros::spin();
}

void ConversionNode::cam_callback(etsi_its_cam_msgs::CAM msg) {
  // TODO
  // auto etsiCAM = convert_CAMtoC(msg);
  // ASN.1-Coding
  // char errbuf[1024];
  // size_t errlen = sizeof(errbuf);
  // int checkRet = asn_check_constraints(&asn_DEF_CAM, &etsiCAM, errbuf, &errlen);
  // if (checkRet)
  // {
  //     ROS_ERROR("Error: %s", errbuf);
  //     return;
  // }
  // asn_encode_to_new_buffer_result_t res = asn_encode_to_new_buffer(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, &etsiCAM);
  // if (res.result.encoded == -1)
  // {
  //     ROS_ERROR("Err: %s", res.result.failed_type->xml_tag);
  // }
  // ROS_INFO("CAM-Length: %i", (int)res.result.encoded);

  // asn_fprint(stdout, &asn_DEF_CAM, &etsiCAM);

  etsi_its_asn1_msgs::ASN1_udp cam_asn1;
  cam_asn1.header.stamp = ros::Time::now();
  cam_asn1.header.frame_id = "base_link";
  // cam_asn1.data = std::vector<uint8_t>((char *)res.buffer, (char *)res.buffer + (int)res.result.encoded);
  cam_asn1_pub_.publish(cam_asn1);
}

void ConversionNode::cam_asn1_callback(etsi_its_asn1_msgs::ASN1_udp msg) {

  CAM_t *camPdu = 0;
  asn_dec_rval_t decodeRet = asn_decode(0, ATS_UNALIGNED_BASIC_PER, &asn_DEF_CAM, (void **)&camPdu, &msg.data[0], msg.data.size());
  asn_fprint(stdout, &asn_DEF_CAM, camPdu);
  auto camROS = etsi_its_cam_conversion::convert_CAMtoRos(*camPdu);
  cam_ros_pub_.publish(camROS);
}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ConversionNode");

  etsi_its_conversion::ConversionNode node;

  return 0;
}

