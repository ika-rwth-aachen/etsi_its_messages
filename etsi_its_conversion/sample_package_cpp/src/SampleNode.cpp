#include <SampleNode.h>


namespace sample_package_cpp {


SampleNode::SampleNode() : node_handle_(), private_node_handle_("~") {
  ROS_INFO("SampleNode starting...");

  // load parameters (defaults and min/max are also handled by dynamic_reconfigure)
  if(!private_node_handle_.param("parameter_float", parameter_float_, 0.0f)) ROS_WARN("parameter_float not set, defaulting to %f.", parameter_float_);
  if(!private_node_handle_.param("parameter_bool", parameter_bool_, false)) ROS_WARN("parameter_bool not set, defaulting to %d.", parameter_bool_);
  if(!private_node_handle_.param<std::string>("parameter_string", parameter_string_, "none")) ROS_WARN("parameter_string not set, defaulting to %s.", parameter_string_.c_str());

  // load parameters that can not be dynamically reconfigured
  if(!private_node_handle_.param("create_publisher", create_publisher_, true)) ROS_WARN("create_publisher not set, defaulting to %d.", create_publisher_);
  if(!private_node_handle_.param("create_subscriber", create_subscriber_, true)) ROS_WARN("create_subscriber not set, defaulting to %d.", create_subscriber_);

  ROS_INFO("Config: parameter_float = %f, parameter_bool = %d, parameter_string = %s, create_publisher = %d, create_subscriber = %d", parameter_float_, parameter_bool_, parameter_string_.c_str(), create_publisher_, create_subscriber_);

  // setup dynamic_reconfigure
  dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<params_SampleNodeConfig>>(private_node_handle_);
  dynamic_reconfigure::Server<params_SampleNodeConfig>::CallbackType config_callback = boost::bind(&SampleNode::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(config_callback);

  // setup publisher and subscriber
  if (create_publisher_) pub_ = private_node_handle_.advertise<std_msgs::String>("message", 1);
  if (create_subscriber_) sub_ = private_node_handle_.subscribe("message", 1, &SampleNode::messageCallback, this);

  // create timer for publishing messages
  if (create_publisher_) timer_ = node_handle_.createTimer(ros::Duration(2.0), &SampleNode::timerCallback, this);

  ros::spin();
}


void SampleNode::dynamicReconfigureCallback(params_SampleNodeConfig &config, uint32_t level) {

  ROS_INFO("dynamic_reconfigure request: parameter_float = %f, parameter_bool = %d, parameter_string = %s", 
            config.parameter_float,
            config.parameter_bool,
            config.parameter_string.c_str());

  parameter_float_ = config.parameter_float;
  parameter_bool_ = config.parameter_bool;
  parameter_string_ = config.parameter_string;
}


void SampleNode::timerCallback(const ros::TimerEvent& event) {

  ROS_INFO("Publishing message at time %f", event.current_real.toSec());

  std_msgs::String msg;
  msg.data = "SampleNode is running";

  pub_.publish(msg);
}


void SampleNode::messageCallback(const std_msgs::String& msg) {

  ROS_INFO("Received message: %s", msg.data.c_str());
}


}  // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "SampleNode");

  sample_package_cpp::SampleNode node;

  return 0;
}

