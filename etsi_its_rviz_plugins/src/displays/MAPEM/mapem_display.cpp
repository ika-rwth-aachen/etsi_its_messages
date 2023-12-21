/** ============================================================================
MIT License

Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#include "displays/MAPEM/mapem_display.hpp"

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
#include "rviz_common/properties/ros_topic_property.hpp"

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

MAPEMDisplay::MAPEMDisplay()
{
  // General Properties
  // spatem_topic_property_ = new rviz_common::properties::RosTopicProperty("SPATEM Topic", "",
  //     "etsi_its_spatem_msgs::msg::SPATEM", "Topic of corresponding SPATEMs", this);
  buffer_timeout_ = new rviz_common::properties::FloatProperty(
    "Timeout", 60.0f,
    "Time (in s) until MAP disappears", this);
  buffer_timeout_->setMin(0);
  // color_property_ = new rviz_common::properties::ColorProperty(
  //   "Color", QColor(25, 0, 255),
  //   "Object color", this);
  // show_meta_ = new rviz_common::properties::BoolProperty("Metadata", true,
  //   "Show metadata as text next to objects", this);

}

MAPEMDisplay::~MAPEMDisplay()
{
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void MAPEMDisplay::onInitialize()
{
  RTDClass::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void MAPEMDisplay::reset()
{
  RTDClass::reset();
  manual_object_->clear();
}

void MAPEMDisplay::processMessage(etsi_its_mapem_msgs::msg::MAPEM::ConstSharedPtr msg)
{
  // Process MAPEM message
  rclcpp::Time now = rviz_node_->now();
  if(!msg->map.time_stamp_is_present) return;
  etsi_its_mapem_msgs::msg::MinuteOfTheYear moy = msg->map.time_stamp;

  // Intersections
  if(!msg->map.intersections_is_present) return;
  for(size_t i = 0; i<msg->map.intersections.array.size(); i++)
  {
    IntersectionRenderObject intsct(msg->map.intersections.array[i], moy, now);
    if(!intsct.validateFloats())
    {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
      return;
    }
    // Check if IntersectionID is already present in list
    auto it = intersections_.find(intsct.getIntersectionID());
    if (it != intersections_.end()) it->second = intsct; // Key exists, update the value
    else intersections_.insert(std::make_pair(intsct.getIntersectionID(), intsct));
  }
  
  return;
}

void MAPEMDisplay::update(float, float)
{
  // Check for outdated intersections
  for (auto it = intersections_.begin(); it != intersections_.end(); ) {
        if (it->second.getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) it = intersections_.erase(it);
        else ++it;
  }

  // Render all valid cams
  bboxs_.clear();
  texts_.clear();
  // for(auto it = cams_.begin(); it != cams_.end(); ++it) {

  //   CAMRenderObject cam = it->second;
  //   Ogre::Vector3 sn_position;
  //   Ogre::Quaternion sn_orientation;
  //   if (!context_->getFrameManager()->getTransform(cam.getHeader(), sn_position, sn_orientation)) {
  //     setMissingTransformToFixedFrame(cam.getHeader().frame_id);
  //     return;
  //   }
  //   setTransformOk();

  //   // set pose of scene node
  //   scene_node_->setPosition(sn_position);
  //   scene_node_->setOrientation(sn_orientation);

  //   auto child_scene_node = scene_node_->createChildSceneNode();
  //   // Set position of scene node
  //   geometry_msgs::msg::Pose pose = cam.getPose();
  //   geometry_msgs::msg::Vector3 dimensions = cam.getDimensions();
  //   Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  //   Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  //   if(3 <= cam.getStationType() && cam.getStationType() <= 11)
  //   {
  //     // If the station type of the originating ITS-S is set to one out of the values 3 to 11
  //     // the reference point shall be the ground position of the centre of the front side of
  //     // the bounding box of the vehicle.
  //     // https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.03.01_30/en_30263702v010301v.pdf
  //     position.x-=dimensions.x/2.0;
  //     position.z+=dimensions.z/2.0;
  //   }

  //   // set pose of child scene node of bounding-box
  //   child_scene_node->setPosition(position);
  //   child_scene_node->setOrientation(orientation);

  //   // create boundind-box object
  //   std::shared_ptr<rviz_rendering::Shape> bbox = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, child_scene_node);

  //   // set the dimensions of bounding box
  //   Ogre::Vector3 dims;
  //   double scale = bb_scale_->getFloat();
  //   dims.x = dimensions.x*scale;
  //   dims.y = dimensions.y*scale;
  //   dims.z = dimensions.z*scale;
  //   bbox->setScale(dims);
  //   // set the color of bounding box
  //   Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
  //   bbox->setColor(bb_color);
  //   bboxs_.push_back(bbox);

  //   // Visualize meta-information as text
  //   if(show_meta_->getBool()) {
  //     std::string text;
  //     if(show_station_id_->getBool()) {
  //       text+="StationID: " + std::to_string(cam.getStationID());
  //       text+="\n";
  //     }
  //     if(show_speed_->getBool()) {
  //       text+="Speed: " + std::to_string((int)(cam.getSpeed()*3.6)) + " km/h";
  //     }
  //     if(!text.size()) return;
  //     std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
  //     double height = dims.z;
  //     height+=text_render->getBoundingRadius();
  //     Ogre::Vector3 offs(0.0, 0.0, height);
  //     // There is a bug in rviz_rendering::MovableText::setGlobalTranslation https://github.com/ros2/rviz/issues/974
  //     text_render->setGlobalTranslation(offs);
  //     Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_->getColor());
  //     text_render->setColor(text_color);
  //     child_scene_node->attachObject(text_render.get());
  //     texts_.push_back(text_render);
  //   }
  // }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::MAPEMDisplay, rviz_common::Display)