#include "displays/CAM/cam_display.hpp"

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

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

CAMDisplay::CAMDisplay()
{
  // General Properties
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Buffer Timeout", 0.1f,
    "Time-Delta until CAM is removed from Buffer.", this);
  buffer_timeout_->setMin(0);
  bb_scale_ = new rviz_common::properties::FloatProperty(
    "Bounding Box Scale", 1.0f,
    "Scaling factor to in/decrease the size of the visualized bounding boxes.", this);
  bb_scale_->setMin(0.01);
  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Color to visualize the CAMs.", this);
  show_meta_ = new rviz_common::properties::BoolProperty("Show Meta-Information", true, 
    "Visualizing CAM Meta-Information", this);
  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(25, 0, 255),
    "Color to visualize the CAMs.", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Char height", 4.0, "Height of characters, ~ Font size", show_meta_);
  show_station_id_ = new rviz_common::properties::BoolProperty("Show StationID", true, 
    "Visualizing CAM StationID", show_meta_);
  show_speed_ = new rviz_common::properties::BoolProperty("Show Speed", true, 
    "Visualizing CAM Speed", show_meta_);
  
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

void CAMDisplay::processMessage(etsi_its_cam_msgs::msg::CAM::ConstSharedPtr msg)
{
  // Check if Station ID is already present in list
  int st_id = etsi_its_cam_msgs::access::getStationID(*msg);
  for(unsigned int i=0; i<cams_.size(); i++) {
    if(st_id == cams_[i].station_id) {
      // Replace existing cam in list
      CAMRenderObject cam(*msg, rviz_node_->now(), 5); // 5 leap seconds in 2023
      if (!cam.validateFloats()) {
        setStatus(
          rviz_common::properties::StatusProperty::Error, "Topic",
          "Message contained invalid floating point values (nans or infs)");
        return;
      }
      cams_[i] = cam;
      return;
    }
  }

  // Station ID seems not to be part of cams_
  // Add to vector
  CAMRenderObject cam(*msg, rviz_node_->now(), 5); // 5 leap seconds in 2023
  if (!cam.validateFloats()) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  cams_.push_back(cam);
  return;
}

void CAMDisplay::update(float wall_dt, float ros_dt)
{
  // Check for outdated CAMs
  unsigned int i=0;
  while(i<cams_.size()) {
    RCLCPP_DEBUG_STREAM(rviz_node_->get_logger(), "cams_[i].getAge(rviz_node_->now()): " << cams_[i].getAge(rviz_node_->now()));
    // To-Do: There seems to be something wrong with the age?
    if(cams_[i].getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) cams_.erase(cams_.begin()+i);
    else i++;
  }

  // Render all valid cams
  bboxs_.clear();
  texts_.clear();
  for(unsigned int j=0; j<cams_.size(); j++) {
    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(cams_[j].header, sn_position, sn_orientation)) {
      setMissingTransformToFixedFrame(cams_[j].header.frame_id);
      return;
    }
    setTransformOk();

    // set pose of scene node
    scene_node_->setPosition(sn_position);
    scene_node_->setOrientation(sn_orientation);

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node
    // If the station type of the originating ITS-S is set to one out of the values 3 to 11
    // the reference point shall be the ground position of the centre of the front side of
    // the bounding box of the vehicle.
    // https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.01_30/en_30263702v010301v.pdf
    Ogre::Vector3 position(cams_[j].pose.position.x-cams_[j].length/2.0, cams_[j].pose.position.y, cams_[j].pose.position.z+cams_[j].height/2.0);
    Ogre::Quaternion orientation(cams_[j].pose.orientation.w, cams_[j].pose.orientation.x, cams_[j].pose.orientation.y, cams_[j].pose.orientation.z);
    // set pose of child scene node of bounding-box
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);
    // create boundind-box object
    std::shared_ptr<rviz_rendering::Shape> bbox = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, child_scene_node);
    // set the dimensions of bounding box
    Ogre::Vector3 dims;
    double scale = bb_scale_->getFloat();
    dims.x = cams_[j].length*scale;
    dims.y = cams_[j].width*scale;
    dims.z = cams_[j].height*scale;
    bbox->setScale(dims);
    // set the color of bounding box
    Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
    bbox->setColor(bb_color);
    bboxs_.push_back(bbox);

    // Visualize meta-information as text
    if(show_meta_->getBool()) {
      std::string text;
      if(show_station_id_->getBool()) {
        text+="StationID: " + std::to_string(cams_[j].station_id);
        text+="\n";
      }
      if(show_speed_->getBool()) {
        text+="Speed: " + std::to_string((int)(cams_[j].speed*3.6)) + " km/h";
      }
      if(!text.size()) return;
      std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
      double height = dims.z;
      height+=text_render->getBoundingRadius();
      Ogre::Vector3 offs(0.0, 0.0, height);
      // Maybe there is a bug in rviz_rendering::MovableText::setGlobalTranslation
      // Currently only the given y-Position is set
      // https://github.com/ros2/rviz/blob/1ac419472ed06cdd52842a8f964f953a75395245/rviz_rendering/src/rviz_rendering/objects/movable_text.cpp#L520
      // Shows that the global_translation-vector is mutliplied with Ogre::Vector3::UNIT_Y is this intended?
      // In the ROS1 implementation the translation-vector is added without any multiplication
      // See: https://github.com/ros-visualization/rviz/blob/ec7ab1b0183244c05fbd2d0d1b8d8f53d8f42f2b/src/rviz/ogre_helpers/movable_text.cpp#L506
      // I've opened an Issue here: https://github.com/ros2/rviz/issues/974
      text_render->setGlobalTranslation(offs);
      Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_->getColor());
      text_render->setColor(text_color);
      child_scene_node->attachObject(text_render.get());
      texts_.push_back(text_render);
    }
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::CAMDisplay, rviz_common::Display)