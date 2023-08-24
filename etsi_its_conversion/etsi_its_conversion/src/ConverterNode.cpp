#include <nodelet/loader.h>
#include <ros/ros.h>


int main(int argc, char **argv) {

  ros::init(argc, argv, "etsi_its_conversion");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(), "etsi_its_conversion/Converter", remap, nargv);

  ros::spin();

  return 0;
}
