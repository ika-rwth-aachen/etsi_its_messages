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
#include "rviz_common/properties/qos_profile_property.hpp"

#include "rviz_common/properties/parse_color.hpp"

namespace etsi_its_msgs
{
namespace displays
{

const Ogre::ColourValue color_grey(0.5, 0.5, 0.5, 1.0);

MAPEMDisplay::MAPEMDisplay() {
  // General Properties
  mapem_timeout_ = new rviz_common::properties::FloatProperty(
    "MAPEM Timeout", 120.0f,
    "Time (in s) until MAP disappears", this);
  mapem_timeout_->setMin(0);
  spatem_topic_property_ = new rviz_common::properties::RosTopicProperty("SPATEM Topic", "/etsi_its_conversion/spatem/out",
      rosidl_generator_traits::data_type<etsi_its_spatem_msgs::msg::SPATEM>(),
      "Topic of corresponding SPATEMs", this, SLOT(changedSPATEMTopic()));
  spatem_qos_property_ = new rviz_common::properties::QosProfileProperty(spatem_topic_property_, qos_profile);
  viz_spatem_ = new rviz_common::properties::BoolProperty("Visualize SPATEMs", false,
    "Show SPATEMs corresponding to received MAPEMs", this, SLOT(changedSPATEMViz()));
  spatem_timeout_ = new rviz_common::properties::FloatProperty(
    "SPATEM Timeout", 0.1f,
    "Time (in s) until SPAT disappears", viz_spatem_);
  spatem_timeout_->setMin(0);
  color_property_ingress_ = new rviz_common::properties::ColorProperty(
    "Ingress Lane Color", QColor(85, 85, 255),
    "Color to visualize Ingress-Lanes", this);
  color_property_egress_ = new rviz_common::properties::ColorProperty(
    "Egress Lane Color", QColor(255, 170, 0),
    "Color to visualize Egress-Lanes", this);
  show_meta_ = new rviz_common::properties::BoolProperty("Metadata", true,
    "Show metadata as text next to MAP reference point", this);
  text_color_property_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255),
    "Text color", show_meta_);
  char_height_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_);


}

MAPEMDisplay::~MAPEMDisplay() {
  if (initialized() ) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void MAPEMDisplay::onInitialize() {
  RTDClass::onInitialize();

  auto nodeAbstraction = context_->getRosNodeAbstraction().lock();
  rviz_node_ = nodeAbstraction->get_raw_node();
  spatem_topic_property_->initialize(nodeAbstraction);
  spatem_qos_property_->initialize(
      [this](rclcpp::QoS profile) {
        spatem_qos_profile_ = profile;
        changedSPATEMTopic();
      });

  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);
}

void MAPEMDisplay::reset() {
  RTDClass::reset();
  manual_object_->clear();
}

void MAPEMDisplay::changedSPATEMViz() {
  if(!viz_spatem_->getBool()) {
    deleteStatus("SPATEM Topic");
    spatem_subscriber_.reset();
  } 
  else changedSPATEMTopic();
}

void MAPEMDisplay::changedSPATEMTopic() {
  spatem_subscriber_.reset();
  received_spats_=0;
  if(spatem_topic_property_->isEmpty()) {
    setStatus(
        rviz_common::properties::StatusProperty::Warn,
        "SPATEM Topic",
        QString("Error subscribing: Empty topic name"));
      return;
  } 

  std::map<std::string, std::vector<std::string>> published_topics = rviz_node_->get_topic_names_and_types();
  bool topic_available = false;
  std::string topic_message_type;
  for (const auto & topic : published_topics) {
    // Only add topics whose type matches.
    if(topic.first == spatem_topic_property_->getTopicStd()) {
      topic_available = true;
      for (const auto & type : topic.second) {
        topic_message_type = type;
        if (type == "etsi_its_spatem_msgs/msg/SPATEM") {
          spatem_subscriber_ = rviz_node_->create_subscription<etsi_its_spatem_msgs::msg::SPATEM>(spatem_topic_property_->getTopicStd(), spatem_qos_profile_, std::bind(&MAPEMDisplay::SPATEMCallback, this, std::placeholders::_1));
          return;
        }
      }
    }
  }
  if(!topic_available) {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "SPATEM Topic",
      QString("Error subscribing to ") + QString::fromStdString(spatem_topic_property_->getTopicStd())
      + QString(": Topic is not available!"));
  }
  else {
    setStatus(
      rviz_common::properties::StatusProperty::Warn, "SPATEM Topic",
      QString("Error subscribing to ") + QString::fromStdString(spatem_topic_property_->getTopicStd())
      + QString(": Message type ") + QString::fromStdString(topic_message_type) + QString::fromStdString(" does not equal etsi_its_spatem_msgs::msg::SPATEM!"));
  }
}

void MAPEMDisplay::SPATEMCallback(etsi_its_spatem_msgs::msg::SPATEM::ConstSharedPtr msg) {

  // iterate over all IntersectionStates
  for(size_t i = 0; i<msg->spat.intersections.array.size(); i++) {
    unsigned int intersection_id = msg->spat.intersections.array[i].id.id.value;
    // Check if IntersectionID is already present in intersections-list
    auto it = intersections_.find(intersection_id);
    if (it == intersections_.end()) continue; // intersection is not available, continue loop
    for(size_t j=0; j<msg->spat.intersections.array[i].states.array.size(); j++) {
      etsi_its_spatem_msgs::msg::MovementState spat_mvmt_state = msg->spat.intersections.array[i].states.array[j];
      IntersectionMovementState mvmt_state;
      // Fill the IntersectionMovementState
      mvmt_state.signal_group_id = spat_mvmt_state.signal_group.value;
      // Check if SignalGroup is already present in IntersectionMovementState of Intersection
      auto mvmnt_it = it->second.movement_states.find(mvmt_state.signal_group_id);
      if (mvmnt_it != it->second.movement_states.end()) mvmnt_it->second = mvmt_state; // SignalGroup exists, update the value
      else it->second.movement_states.insert(std::make_pair(mvmt_state.signal_group_id, mvmt_state));
    }
  }

  ++received_spats_;
  setStatus(
    rviz_common::properties::StatusProperty::Ok, "SPATEM Topic", QString::number(received_spats_) + " messages received");
}

void MAPEMDisplay::processMessage(etsi_its_mapem_msgs::msg::MAPEM::ConstSharedPtr msg) {
  // Process MAPEM message
  rclcpp::Time now = rviz_node_->now();
  // Intersections
  if(!msg->map.intersections_is_present) return;
  etsi_its_mapem_msgs::msg::MinuteOfTheYear moy = msg->map.time_stamp;
  for(size_t i = 0; i<msg->map.intersections.array.size(); i++)
  {
    IntersectionRenderObject intsct(msg->map.intersections.array[i], msg->map.time_stamp_is_present, moy, now);
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

void MAPEMDisplay::update(float, float) {
  // Check for outdated intersections
  for (auto it = intersections_.begin(); it != intersections_.end(); ) {
        if (it->second.getAge(rviz_node_->now()) > mapem_timeout_->getFloat()) it = intersections_.erase(it);
        else ++it;
  }
  // Render all valid intersections
  intsct_ref_points_.clear();
  lane_lines_.clear();
  signal_groups_.clear();
  texts_.clear();
  for(auto it = intersections_.begin(); it != intersections_.end(); ++it) {
    IntersectionRenderObject intsctn = it->second;
    Ogre::Vector3 sn_position;
    Ogre::Quaternion sn_orientation;
    if (!context_->getFrameManager()->getTransform(intsctn.getHeader(), sn_position, sn_orientation)) {
      setMissingTransformToFixedFrame(intsctn.getHeader().frame_id);
      return;
    }
    setTransformOk();

    // set pose of scene node
    scene_node_->setPosition(sn_position);
    scene_node_->setOrientation(sn_orientation);

    auto child_scene_node = scene_node_->createChildSceneNode();
    // Set position of scene node
    geometry_msgs::msg::Point ref_position = intsctn.getRefPosition();
    Ogre::Vector3 position(ref_position.x, ref_position.y, ref_position.z);
    tf2::Quaternion rot_offset = intsctn.getGridConvergenceQuaternion();
    Ogre::Quaternion orientation(rot_offset.w(), rot_offset.x(), rot_offset.y(), rot_offset.z());

    // set pose of child scene node of intersection
    child_scene_node->setPosition(position);
    child_scene_node->setOrientation(orientation);

    // create sphere object
    std::shared_ptr<rviz_rendering::Shape> sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, child_scene_node);

    // set the dimensions of sphere
    Ogre::Vector3 dims;
    dims.x = 1.0;
    dims.y = 1.0;
    dims.z = 1.0;
    sphere->setScale(dims);

    // set the color of sphere
    sphere->setColor(color_grey);
    intsct_ref_points_.push_back(sphere);

    // visualize the lanes
    for(size_t i = 0; i<intsctn.lanes.size(); i++)
    {
      std::shared_ptr<rviz_rendering::BillboardLine> line = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, child_scene_node);
      Ogre::ColourValue lane_color;
      if(intsctn.lanes[i].direction == LaneDirection::ingress) lane_color = rviz_common::properties::qtToOgre(color_property_ingress_->getColor());
      else if(intsctn.lanes[i].direction == LaneDirection::egress) lane_color = rviz_common::properties::qtToOgre(color_property_egress_->getColor());
      else lane_color = color_grey;
      line->setColor(lane_color.r, lane_color.g, lane_color.b, lane_color.a);
      line->setLineWidth(1.0);
      for(size_t j = 0; j<intsctn.lanes[i].nodes.size(); j++) {
        Ogre::Vector3 p;
        p.x = intsctn.lanes[i].nodes[j].x;
        p.y = intsctn.lanes[i].nodes[j].y;
        p.z = intsctn.lanes[i].nodes[j].z;
        line->addPoint(p);
      }
      lane_lines_.push_back(line);
      // Signal Groups
      if(viz_spatem_->getBool() && intsctn.lanes[i].signal_group_ids.size()) {// && intsctn.lanes[i].direction == LaneDirection::ingress) {
        std::shared_ptr<rviz_rendering::Shape> sg = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, child_scene_node);
        sg->setScale(dims);
        sg->setColor(color_grey);
        Ogre::Vector3 p;
        p.x = intsctn.lanes[i].nodes.front().x;
        p.y = intsctn.lanes[i].nodes.front().y;
        p.z = intsctn.lanes[i].nodes.front().z;
        sg->setPosition(p);
        signal_groups_.push_back(sg);
      }
    }

    // Visualize meta-information as text
    if(show_meta_->getBool()) {
      std::string text;
      text+="IntersectionID: " + std::to_string(intsctn.getIntersectionID());
      std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_->getFloat());
      double height = dims.z;
      height+=text_render->getBoundingRadius();
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

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::MAPEMDisplay, rviz_common::Display)