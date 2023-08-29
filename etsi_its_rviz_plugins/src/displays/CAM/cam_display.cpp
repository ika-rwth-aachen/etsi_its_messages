#include "etsi_its_msgs/displays/CAM/cam_display.hpp"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

CAMDisplay::CAMDisplay()
{
  // General Properties
  // color_property_ = new rviz_common::properties::ColorProperty(
  //   "Color", QColor(255, 0, 25),
  //   "Color to visualize the Ego-Vehicle.", this);
  // alpha_property_ = new rviz_common::properties::FloatProperty(
  //   "Alpha", 0.5f,
  //   "Amount of transparency to apply.", this);
  // viz_bounding_box_ = new rviz_common::properties::BoolProperty("Bounding box", true, 
  //   "Visualize the bounding box of the Ego-Vehicle.", this);
  // viz_direction_ind_ = new rviz_common::properties::BoolProperty("Orientation", true, 
  //   "Visualize the direction indicator of the Ego-Vehicle.", this);
  // viz_velocity_ = new rviz_common::properties::BoolProperty("Velocity arrow", true, 
  //   "Add an arrow visualizing the EgoVehicles's velocity", this);
  // viz_acceleration_ = new rviz_common::properties::BoolProperty("Acceleration arrow", false, 
  //   "Add an arrow visualizing the EgoVehicles's acceleration", this);
  // viz_text_ = new rviz_common::properties::BoolProperty("Text information", false, 
  //   "Visualize informing text about the Ego-Vehicle.", this);

  // // Velocity options
  // velocity_scale_ = new rviz_common::properties::FloatProperty("Velocity scale", 1.0, "Scale the length of the velocity arrows", viz_velocity_);
  // use_velocity_color_ = new rviz_common::properties::BoolProperty("Use velocity color", true, 
  //   "Visualize the velocity arrow in the bbox color. If not set, use specific color instead.", viz_velocity_);
  // velocity_color_property_ = new rviz_common::properties::ColorProperty(
  //   "Velocity Color", QColor(255, 0, 255),
  //   "Color to visualize velocity arrow", viz_velocity_);

  // // Acceleration options
  // acceleration_scale_ = new rviz_common::properties::FloatProperty("Acceleration scale", 10.0, "Scale the length of the acceleration arrows", viz_acceleration_);
  // use_acceleration_color_ = new rviz_common::properties::BoolProperty("Use acceleration color", true, 
  //   "Visualize the acceleration arrow in the bbox color. If not set, use specific color instead.", viz_acceleration_);
  // acceleration_color_property_ = new rviz_common::properties::ColorProperty(
  //   "Acceleration Color", QColor(255, 0, 0),
  //   "Color to visualize acceleration arrow", viz_acceleration_);

  // // Text printing options
  // char_height_ = new rviz_common::properties::FloatProperty("Char height", 4.0,
  //   "Height of characters, ~ Font size", viz_text_);
  // print_vel_ = new rviz_common::properties::BoolProperty("Velocity", true, 
  //   "Print the speed of the Ego-Vehicle within text.", viz_text_);

  // alpha_property_->setMin(0);
  // alpha_property_->setMax(1);

}

CAMDisplay::~CAMDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void CAMDisplay::onInitialize()
{
  RTDClass::onInitialize();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void CAMDisplay::reset()
{
  RTDClass::reset();
  manual_object_->clear();
}

bool validateFloats(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg)
{
  bool valid = true;
  // ToDo
  return valid;
}

void CAMDisplay::processMessage(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg)
{
  if (!validateFloats(msg)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Ogre::Vector3 position;
  // Ogre::Quaternion orientation;
  // if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
  //   setMissingTransformToFixedFrame(msg->header.frame_id);
  //   return;
  // }
  // setTransformOk();

  // scene_node_->setPosition(position);
  // scene_node_->setOrientation(orientation);

  // Set Colors
  // Ogre::ColourValue color_general = rviz_common::properties::qtToOgre(color_property_->getColor());
  // Ogre::ColourValue color_text = rviz_common::properties::qtToOgre(color_property_->getColor());
 
  // color_general.a = alpha_property_->getFloat();
  // color_text.a = alpha_property_->getFloat();

  // bool visualize_bounding_box = viz_bounding_box_->getBool();
  // bool visualize_direction_indicator = viz_direction_ind_->getBool();
  // bool visualize_velocity = viz_velocity_->getBool();
  // float velocity_scale;
  // bool use_velocity_color;
  // Ogre::ColourValue velocity_color;
  // if(visualize_velocity)
  // {
  //   velocity_scale = velocity_scale_->getFloat();
  //   use_velocity_color = use_velocity_color_->getBool();
  //   velocity_color = rviz_common::properties::qtToOgre(velocity_color_property_->getColor());
  //   velocity_color.a = alpha_property_->getFloat(); 
  // }

  // bool visualize_acceleration = viz_acceleration_->getBool();
  // float acceleration_scale;
  // bool use_acceleration_color;
  // Ogre::ColourValue acceleration_color;
  // if(visualize_acceleration)
  // {
  //   acceleration_scale = acceleration_scale_->getFloat();
  //   use_acceleration_color = use_acceleration_color_->getBool();
  //   acceleration_color = rviz_common::properties::qtToOgre(acceleration_color_property_->getColor());
  //   acceleration_color.a = alpha_property_->getFloat(); 
  // }

  // bool visualize_text = viz_text_->getBool();
  // float char_height = char_height_->getFloat();
  // bool print_vel = false;
  // if(visualize_text) {
  //   print_vel = print_vel_->getBool();
  // }

  // manual_object_->clear();

  // Render Bounding Box
  // ToDo
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::CAMDisplay, rviz_common::Display)