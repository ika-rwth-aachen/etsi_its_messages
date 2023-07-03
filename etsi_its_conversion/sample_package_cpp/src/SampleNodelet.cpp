#include <pluginlib/class_list_macros.h>

#include <SampleNodelet.h>


PLUGINLIB_EXPORT_CLASS(sample_package_cpp::SampleNodelet, nodelet::Nodelet)


namespace sample_package_cpp {


SampleNodelet::SampleNodelet() {}


SampleNodelet::~SampleNodelet() {
  NODELET_INFO("SampleNodelet stopped");
}


void SampleNodelet::onInit() {

  // NodeHandles cannot be used before this point
  node_handle_ = this->getMTNodeHandle();
  private_node_handle_ = this->getMTPrivateNodeHandle();
  NODELET_INFO("SampleNodelet starting...");

  // load parameters (defaults and min/max are also handled by dynamic_reconfigure)
  if(!private_node_handle_.param("parameter_float", parameter_float_, 0.0f)) NODELET_WARN("parameter_float not set, defaulting to %f.", parameter_float_);
  if(!private_node_handle_.param("parameter_bool", parameter_bool_, false)) NODELET_WARN("parameter_bool not set, defaulting to %d.", parameter_bool_);
  if(!private_node_handle_.param<std::string>("parameter_string", parameter_string_, "none")) NODELET_WARN("parameter_string not set, defaulting to %s.", parameter_string_.c_str());

  // load parameters that can not be dynamically reconfigured
  if(!private_node_handle_.param("create_publisher", create_publisher_, true)) ROS_WARN("create_publisher not set, defaulting to %d.", create_publisher_);
  if(!private_node_handle_.param("create_subscriber", create_subscriber_, true)) ROS_WARN("create_subscriber not set, defaulting to %d.", create_subscriber_);

  NODELET_INFO("Config: parameter_float = %f, parameter_bool = %d, parameter_string = %s, create_publisher = %d, create_subscriber = %d", parameter_float_, parameter_bool_, parameter_string_.c_str(), create_publisher_, create_subscriber_);


  // setup dynamic_reconfigure
  dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<params_SampleNodeletConfig>>(private_node_handle_);
  dynamic_reconfigure::Server<params_SampleNodeletConfig>::CallbackType config_callback = boost::bind(&SampleNodelet::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(config_callback);

  // setup publisher and subscriber
  if (create_publisher_) pub_ = private_node_handle_.advertise<std_msgs::String>("message", 1);
  if (create_subscriber_) sub_ = private_node_handle_.subscribe("message", 1, &SampleNodelet::messageCallback, this);

  // create timer for publishing messages
  if (create_publisher_) timer_ = node_handle_.createTimer(ros::Duration(2.0), &SampleNodelet::timerCallback, this);
}


void SampleNodelet::dynamicReconfigureCallback(params_SampleNodeletConfig &config, uint32_t level) {

  ROS_INFO("dynamic_reconfigure request: parameter_float = %f, parameter_bool = %d, parameter_string = %s", 
            config.parameter_float,
            config.parameter_bool,
            config.parameter_string.c_str());

  parameter_float_ = config.parameter_float;
  parameter_bool_ = config.parameter_bool;
  parameter_string_ = config.parameter_string;
}


void SampleNodelet::timerCallback(const ros::TimerEvent& event) {

  NODELET_INFO("Publishing message at time %f", event.current_real.toSec());

  // messages must be published as pointers in nodelets
  std_msgs::StringPtr msg = std_msgs::StringPtr(new std_msgs::String);
  msg->data = "SampleNodelet is running";

  pub_.publish(msg);
}


void SampleNodelet::messageCallback(const std_msgs::StringConstPtr msg) {

  // DO NOT change the received message, it is shared among all other nodelets!
  NODELET_INFO("Received message: %s", msg->data.c_str());
}


}  // end of namespace
