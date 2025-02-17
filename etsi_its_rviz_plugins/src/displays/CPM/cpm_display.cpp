/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include "displays/CPM/cpm_display.hpp"

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs {
namespace displays {

CPMDisplay::CPMDisplay() {
  // General Properties
  buffer_timeout_ =
      new rviz_common::properties::FloatProperty("Timeout", 1.0f, "Time period (in s) in which CPM are valid and should be displayed (now - reference_time of CPM)", this);
  buffer_timeout_->setMin(0);
  bb_scale_ = new rviz_common::properties::FloatProperty("Scale", 1.0f, "Scale of objects", this);
  bb_scale_->setMin(0.01);
  color_property_ = new rviz_common::properties::ColorProperty("Color", QColor(25, 0, 255), "Object color", this);
  show_meta_ =
      new rviz_common::properties::BoolProperty("Metadata", true, "Show metadata as text next to objects", this);
  text_color_property_ =
      new rviz_common::properties::ColorProperty("Color", QColor(25, 0, 255), "Text color", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Scale", 0.5, "Scale of text", show_meta_);
  show_station_id_ = new rviz_common::properties::BoolProperty("StationID", true, "Show StationID", show_meta_);
  show_speed_ = new rviz_common::properties::BoolProperty("Speed", true, "Show speed", show_meta_);
}

CPMDisplay::~CPMDisplay() {
  if (initialized()) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void CPMDisplay::onInitialize() {
  RTDClass::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void CPMDisplay::reset() {
  RTDClass::reset();
  manual_object_->clear();
}

void CPMDisplay::processMessage(etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage::ConstSharedPtr msg) {
  rclcpp::Time now = rviz_node_->now();
  uint64_t nanosecs = now.nanoseconds();
  if (nanosecs == 0) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Topic", "Message received before clock got a valid time");
    return;
  }

  CPMRenderObject cpm(*msg);

  if (!cpm.validateFloats()) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Check if Station ID is already present in list
  auto it = cpms_.find(cpm.getStationID());
  if (it != cpms_.end()) {
    it->second = cpm;  // Key exists, update the value
  } else {
    cpms_.insert(std::make_pair(cpm.getStationID(), cpm));
  }

  return;
}

void CPMDisplay::update(float, float) {
  // Check for outdated CPMs
  for (auto it = cpms_.begin(); it != cpms_.end();) {
    if (it->second.getAge(rviz_node_->now()) > buffer_timeout_->getFloat()) {
      it = cpms_.erase(it);
    } else {
      ++it;
    }
  }
  bboxs_.clear();
  texts_.clear();
  for (auto it = cpms_.begin(); it != cpms_.end(); ++it) {
    CPMRenderObject cpm = it->second;
    for (int i = 0; i < cpm.getNumberOfObjects(); i++) {
      Ogre::Vector3 sn_position;
      Ogre::Quaternion sn_orientation;
      if (!context_->getFrameManager()->getTransform(cpm.getHeader(), sn_position, sn_orientation)) {
        // Check if transform exists
        setMissingTransformToFixedFrame(cpm.getHeader().frame_id);
        return;
      }
      // We don't want to use the transform in sn_position and sn_orientation though, because they are only in float precision.
      // So we get the transfrom manually from tf2:
      std::string fixed_frame = fixed_frame_.toStdString();
      geometry_msgs::msg::PoseStamped pose_origin;
      pose_origin.header = cpm.getHeader();
      pose_origin.pose.position.x = 0;
      pose_origin.pose.position.y = 0;
      pose_origin.pose.position.z = 0;
      pose_origin.pose.orientation.w = 1;
      pose_origin.pose.orientation.x = 0;
      pose_origin.pose.orientation.y = 0;
      pose_origin.pose.orientation.z = 0;
      geometry_msgs::msg::PoseStamped pose_fixed_frame =
          context_->getTransformationManager()->getCurrentTransformer()->transform(pose_origin, fixed_frame);
      geometry_msgs::msg::TransformStamped transform_to_fixed_frame;
      transform_to_fixed_frame.header = pose_fixed_frame.header;
      transform_to_fixed_frame.transform.translation.x = pose_fixed_frame.pose.position.x;
      transform_to_fixed_frame.transform.translation.y = pose_fixed_frame.pose.position.y;
      transform_to_fixed_frame.transform.translation.z = pose_fixed_frame.pose.position.z;
      transform_to_fixed_frame.transform.rotation = pose_fixed_frame.pose.orientation;

      setTransformOk();

      // set pose of scene node to origin (=fixed frame)!
      scene_node_->setPosition(Ogre::Vector3{0.0f, 0.0f, 0.0f});
      scene_node_->setOrientation(Ogre::Quaternion{1.0f, 0.0f, 0.0f, 0.0f});

      auto child_scene_node = scene_node_->createChildSceneNode();
      // Set position of scene node to the position relative to the fixed frame
      geometry_msgs::msg::Pose pose = cpm.getPoseOfObject(i);

      geometry_msgs::msg::Vector3 dimensions = cpm.getDimensionsOfObject(i);
      tf2::doTransform(pose, pose, transform_to_fixed_frame);
      Ogre::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
      Ogre::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

      // set pose of child scene node of bounding-box
      child_scene_node->setPosition(position);
      child_scene_node->setOrientation(orientation);

      // create bounding-box object
      std::shared_ptr<rviz_rendering::Shape> bbox =
          std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Cube, scene_manager_, child_scene_node);

      // set the dimensions of bounding box
      Ogre::Vector3 dims;
      double scale = bb_scale_->getFloat();
      dims.x = dimensions.x * scale;
      dims.y = dimensions.y * scale;
      dims.z = dimensions.z * scale;
      bbox->setScale(dims);
      // set the color of bounding box
      Ogre::ColourValue bb_color = rviz_common::properties::qtToOgre(color_property_->getColor());
      bbox->setColor(bb_color);
      bboxs_.push_back(bbox);

      // Visualize meta-information as text
      if (show_meta_->getBool()) {
        std::string text;
        if (show_station_id_->getBool()) {
          text += "StationID: " + std::to_string(cpm.getStationID());
          text += "\n";
        }
        if (show_speed_->getBool()) {
          geometry_msgs::msg::Vector3 velocity = cpm.getVelocityOfObject(i);
          //get magnitude of velocity
          double speed = sqrt(pow(velocity.x, 2) + pow(velocity.y, 2) + pow(velocity.z, 2));
          text += "Speed: " + std::to_string((int)(speed * 3.6)) + " km/h";
        }
        if (!text.size()) return;
        std::shared_ptr<rviz_rendering::MovableText> text_render =
            std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
        double height = dims.z;
        height += text_render->getBoundingRadius();
        Ogre::Vector3 offs(0.0, 0.0, height);
        // There is a bug in rviz_rendering::MovableText::setGlobalTranslation https://github.com/ros2/rviz/issues/974
        text_render->setGlobalTranslation(offs);
        Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_->getColor());
        text_render->setColor(text_color);
        child_scene_node->attachObject(text_render.get());
        texts_.push_back(text_render);
      }
    }
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::CPMDisplay, rviz_common::Display)