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

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

CAMDisplay::CAMDisplay()
{
  // General Properties
  buffer_cams_ = new rviz_common::properties::BoolProperty("Buffer CAM's", true, 
    "Buffer multiple CAM's with different station ID's.", this);
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Buffer Timeout", 0.1f,
    "Time-Delta until CAM is removed from Buffer.", buffer_cams_);
  buffer_timeout_->setMin(0);
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 0, 25),
    "Color to visualize the CAMs.", this);

  
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

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

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

  // Check if Station ID is already present
  int st_id = etsi_its_cam_msgs::access::getStationID(*msg);
  for(unsigned int i=0; i<cams_.size(); i++)
  {
    if(st_id == cams_[i].station_id)
    {
      // Replace
      CAMRenderObject cam(*msg, rviz_node_->now(), 5); // 5 leap seconds in 2023
      cams_[i] = cam;
      return;
    }
  }

  // Station ID seems not to be part of cams_
  // Add to vector
  CAMRenderObject cam(*msg, rviz_node_->now(), 5); // 5 leap seconds in 2023
  cams_.push_back(cam);

}

void CAMDisplay::update(float wall_dt, float ros_dt)
{
  // Check for outdated CAMs
  unsigned int i=0;
  while(i<cams_.size())
  {
    if(cams_[i].getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) cams_.erase(cams_.begin()+i);
    else i++;
  }

  // Render all valid cams
  bboxs_.clear();
  for(unsigned int j=0; j<cams_.size(); j++)
  {
    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(cams_[j].header, sn_position, sn_orientation)) {
      setMissingTransformToFixedFrame(cams_[j].header.frame_id);
      return;
    }
    setTransformOk();

    scene_node_->setPosition(sn_position);
    scene_node_->setOrientation(sn_orientation);

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node
    Ogre::Vector3 position(cams_[j].pose.position.x-cams_[j].length/2.0, cams_[j].pose.position.y, cams_[j].pose.position.z);
    Ogre::Quaternion orientation(cams_[j].pose.orientation.w, cams_[j].pose.orientation.x, cams_[j].pose.orientation.y, cams_[j].pose.orientation.z);
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);

    std::shared_ptr<rviz_rendering::Shape> bbox = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, child_scene_node);
    Ogre::Vector3 dims;
    dims.x = cams_[j].length*100;
    dims.y = cams_[j].width*100;
    dims.z = cams_[j].height*100;
    bbox->setScale(dims);
    Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
    bbox->setColor(bb_color);
    bboxs_.push_back(bbox);
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::CAMDisplay, rviz_common::Display)