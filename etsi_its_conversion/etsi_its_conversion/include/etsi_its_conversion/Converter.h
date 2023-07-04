#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <bitstring_msgs/UInt8ArrayStamped.h>
#include <etsi_its_cam_conversion/convertCAM.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>


namespace etsi_its_conversion {


class Converter : public nodelet::Nodelet {

  public:

    virtual void onInit();

  protected:

    void asn1CallbackCam(const bitstring_msgs::UInt8ArrayStamped::ConstPtr bitstring_msg);

    void rosCallbackCam(const etsi_its_cam_msgs::CAM::ConstPtr msg);

  protected:

    static const std::string kInputTopicAsn1Cam;
    static const std::string kOutputTopicCam;
    static const std::string kInputTopicCam;
    static const std::string kOutputTopicAsn1Cam;

    ros::NodeHandle private_node_handle_;

    std::unordered_map<std::string, ros::Subscriber> subscribers_asn1_;
    std::unordered_map<std::string, ros::Publisher> publishers_;
    std::unordered_map<std::string, ros::Subscriber> subscribers_;
    std::unordered_map<std::string, ros::Publisher> publishers_asn1_;
};


}
