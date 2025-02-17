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

#include "displays/MAPEM/mapem_display.hpp"

#include <etsi_its_msgs_utils/spatem_ts_access.hpp>

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
const Ogre::ColourValue color_green(0.18, 0.79, 0.21, 1.0);
const Ogre::ColourValue color_orange(0.9, 0.7, 0.09, 1.0);
const Ogre::ColourValue color_red(0.8, 0.2, 0.2, 1.0);

MAPEMDisplay::MAPEMDisplay() {
  // General Properties
  spatem_topic_property_ = new rviz_common::properties::RosTopicProperty("SPATEM Topic", "/etsi_its_conversion/spatem_ts/out",
      rosidl_generator_traits::data_type<etsi_its_spatem_ts_msgs::msg::SPATEM>(),
      "Topic of corresponding SPATEMs", this, SLOT(changedSPATEMTopic()));
  spatem_qos_property_ = new rviz_common::properties::QosProfileProperty(spatem_topic_property_, qos_profile);
  
  // MAPEM
  viz_mapem_ = new rviz_common::properties::BoolProperty("Visualize MAPEMs", true,
    "Show MAPEMs", this);

  mapem_timeout_ = new rviz_common::properties::FloatProperty(
    "MAPEM Timeout", 120.0f,
    "Time (in s) until MAP disappears", viz_mapem_);
  mapem_timeout_->setMin(0);

  color_property_ingress_ = new rviz_common::properties::ColorProperty(
    "Ingress Lane Color", QColor(85, 85, 255),
    "Color to visualize Ingress-Lanes", viz_mapem_);
  color_property_egress_ = new rviz_common::properties::ColorProperty(
    "Egress Lane Color", QColor(255, 170, 0),
    "Color to visualize Egress-Lanes", viz_mapem_);
  lane_width_property_ = new rviz_common::properties::FloatProperty(
    "MAPEM Lane Width", 1.0, "Width of MAPEM-Lanes", viz_mapem_);
  lane_width_property_->setMin(0.1);
  show_meta_mapem_ = new rviz_common::properties::BoolProperty("Metadata", true,
    "Show metadata as text next to MAP reference point", viz_mapem_);
  text_color_property_mapem_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255),
    "Text color", show_meta_mapem_);
  char_height_mapem_ = new rviz_common::properties::FloatProperty("Scale", 4.0, "Scale of text", show_meta_mapem_);

  // SPATEM
  viz_spatem_ = new rviz_common::properties::BoolProperty("Visualize SPATEMs", false,
    "Show SPATEMs corresponding to received MAPEMs", this, SLOT(changedSPATEMViz()));
  
  spatem_timeout_ = new rviz_common::properties::FloatProperty(
    "SPATEM Timeout", 0.1f,
    "Time (in s) until SPAT disappears", viz_spatem_);
  spatem_timeout_->setMin(0);

  spatem_sphere_scale_property_ = new rviz_common::properties::FloatProperty(
    "SPATEM Sphere Scale", 1.0f,
    "Scaling factor to adjuste size of SPATEM spheres", viz_spatem_);
  spatem_sphere_scale_property_->setMin(0.1);

  show_meta_spatem_ = new rviz_common::properties::BoolProperty("Metadata", true,
    "Show metadata as text next to SPATEM reference point", viz_spatem_);

  text_color_property_spatem_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(255, 255, 255),
    "Text color", show_meta_spatem_);
  char_height_spatem_ = new rviz_common::properties::FloatProperty("Scale", 1.0, "Scale of text", show_meta_spatem_);

  show_spatem_start_time = new rviz_common::properties::BoolProperty("Start time", false,
    "Show SPATEM start time", show_meta_spatem_);

  show_spatem_min_end_time = new rviz_common::properties::BoolProperty("Min end time", true,
    "Show SPATEM min end time", show_meta_spatem_);

  show_spatem_max_end_time = new rviz_common::properties::BoolProperty("Max end time", true,
    "Show SPATEM max end time", show_meta_spatem_);

  show_spatem_likely_time = new rviz_common::properties::BoolProperty("Likely time", false,
    "Show SPATEM likely time", show_meta_spatem_);

  show_spatem_confidence = new rviz_common::properties::BoolProperty("Confidence", false,
    "Show SPATEM confidence", show_meta_spatem_);

  show_spatem_next_time = new rviz_common::properties::BoolProperty("Next time", false,
    "Show SPATEM next time", show_meta_spatem_);

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
        if (type == "etsi_its_spatem_ts_msgs/msg/SPATEM") {
          spatem_subscriber_ = rviz_node_->create_subscription<etsi_its_spatem_ts_msgs::msg::SPATEM>(spatem_topic_property_->getTopicStd(), spatem_qos_profile_, std::bind(&MAPEMDisplay::SPATEMCallback, this, std::placeholders::_1));
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
      + QString(": Message type ") + QString::fromStdString(topic_message_type) + QString::fromStdString(" does not equal etsi_its_spatem_ts_msgs::msg::SPATEM!"));
  }
}

void MAPEMDisplay::SPATEMCallback(etsi_its_spatem_ts_msgs::msg::SPATEM::ConstSharedPtr msg) {
  rclcpp::Time now = rviz_node_->now();
  // iterate over all IntersectionStates
  for(size_t i = 0; i<msg->spat.intersections.array.size(); i++) {
    etsi_its_spatem_ts_msgs::msg::IntersectionState intersection_state = msg->spat.intersections.array[i];
    unsigned int intersection_id = etsi_its_spatem_ts_msgs::access::getIntersectionID(intersection_state);
    // Check if IntersectionID is already present in intersections-list
    auto it = intersections_.find(intersection_id);
    if (it == intersections_.end()) continue; // intersection is not available, continue loop
    // derive stamp from Intersection State
    std_msgs::msg::Header header;
    if(intersection_state.moy_is_present) {
      etsi_its_spatem_ts_msgs::msg::MinuteOfTheYear moy = etsi_its_spatem_ts_msgs::access::getMinuteOfTheYear(intersection_state);
      uint64_t nanosecs = etsi_its_spatem_ts_msgs::access::getUnixNanosecondsFromMinuteOfTheYear(moy, now.nanoseconds());
      if(intersection_state.time_stamp_is_present) {
        double secs_in_minute = etsi_its_spatem_ts_msgs::access::getDSecondValue(intersection_state);
        nanosecs += (uint64_t)(secs_in_minute*1e9);
      }
      header.stamp = rclcpp::Time(nanosecs);
    }
    else {
      header.stamp = now;
    }
    // iterate over all MovemenStates
    for(size_t j=0; j<intersection_state.states.array.size(); j++) {
      etsi_its_spatem_ts_msgs::msg::MovementState spat_mvmt_state = intersection_state.states.array[j];
      IntersectionMovementState mvmt_state;
      // Fill the IntersectionMovementState
      mvmt_state.signal_group_id = etsi_its_spatem_ts_msgs::access::getSignalGroupID(spat_mvmt_state);
      mvmt_state.header = header;
      if(spat_mvmt_state.state_time_speed.array.size()) {
        mvmt_state.phase_state = etsi_its_spatem_ts_msgs::access::getCurrentMovementPhaseState(spat_mvmt_state);

        auto movement_event = etsi_its_spatem_ts_msgs::access::getCurrentMovementEvent(spat_mvmt_state);
        if (movement_event.timing_is_present) {
          mvmt_state.time_change_details = std::make_shared<etsi_its_spatem_ts_msgs::msg::TimeChangeDetails>(movement_event.timing);
        }
      }
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

void MAPEMDisplay::processMessage(etsi_its_mapem_ts_msgs::msg::MAPEM::ConstSharedPtr msg) {
  // Process MAPEM message
  rclcpp::Time now = rviz_node_->now();
  // Intersections
  if(!msg->map.intersections_is_present) return;
  etsi_its_mapem_ts_msgs::msg::MinuteOfTheYear moy = msg->map.time_stamp;
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
    if (it != intersections_.end()) {
      // Intersection exists, update the intersection but keep the MovementStates
      intsct.movement_states = it->second.movement_states;
      it->second = intsct; 
    } 
    else intersections_.insert(std::make_pair(intsct.getIntersectionID(), intsct));
  }
  
  return;
}

void MAPEMDisplay::RenderMapemShapes(Ogre::SceneNode *child_scene_node) {
  // create sphere object
  std::shared_ptr<rviz_rendering::Shape> sphere = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, child_scene_node);

  // set the dimensions of sphere
  double scale = spatem_sphere_scale_property_->getFloat();
  Ogre::Vector3 dims;
  dims.x = 1.0 * scale;
  dims.y = 1.0 * scale;
  dims.z = 1.0 * scale;
  sphere->setScale(dims);

  // set the color of sphere
  sphere->setColor(color_grey);
  intsct_ref_points_.push_back(sphere);
}

void MAPEMDisplay::RenderMapemShapesLane(Ogre::SceneNode *child_scene_node, IntersectionLane& lane) {
  // visualize the lanes
  std::shared_ptr<rviz_rendering::BillboardLine> line = std::make_shared<rviz_rendering::BillboardLine>(scene_manager_, child_scene_node);
  Ogre::ColourValue lane_color;
  if(lane.direction == LaneDirection::ingress) lane_color = rviz_common::properties::qtToOgre(color_property_ingress_->getColor());
  else if(lane.direction == LaneDirection::egress) lane_color = rviz_common::properties::qtToOgre(color_property_egress_->getColor());
  else lane_color = color_grey;
  line->setColor(lane_color.r, lane_color.g, lane_color.b, lane_color.a);
  double line_width = lane_width_property_->getFloat();
  line->setLineWidth(line_width);
  for(size_t j = 0; j<lane.nodes.size(); j++) {
    Ogre::Vector3 p;
    p.x = lane.nodes[j].x;
    p.y = lane.nodes[j].y;
    p.z = lane.nodes[j].z;
    line->addPoint(p);
  }
  lane_lines_.push_back(line); 
}

void MAPEMDisplay::RenderMapemTexts(Ogre::SceneNode *child_scene_node, IntersectionRenderObject& intsctn) {
  std::string text;
  text+="IntersectionID: " + std::to_string(intsctn.getIntersectionID());
  std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text, "Liberation Sans", char_height_mapem_->getFloat());
  double height = spatem_sphere_scale_property_->getFloat();
  height+=text_render->getBoundingRadius();
  
  Ogre::Vector3 offs(0.0, 0.0, height);
  text_render->setGlobalTranslation(offs);
  Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_mapem_->getColor());
  text_render->setColor(text_color);
  child_scene_node->attachObject(text_render.get());
  texts_.push_back(text_render);
}

void MAPEMDisplay::RenderSpatemShapes(Ogre::SceneNode *child_scene_node, IntersectionLane& lane, IntersectionMovementState* intersection_movement_state) {
  // Signal Groups
  if(viz_spatem_->getBool() && lane.signal_group_ids.size() && lane.direction != LaneDirection::egress) {
        
    // create graphical circle to display current movement state phase
    std::shared_ptr<rviz_rendering::Shape> sg = std::make_shared<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, child_scene_node);
          
    // set the dimensions of sphere
    double scale = spatem_sphere_scale_property_->getFloat();
    Ogre::Vector3 dims;
    dims.x = 1.0 * scale;
    dims.y = 1.0 * scale;
    dims.z = 1.0 * scale;
          
    sg->setScale(dims);

    // Set color according to state
    if(intersection_movement_state != nullptr) {
      std::array<float, 4> color = etsi_its_spatem_ts_msgs::access::interpretMovementPhaseStateAsColor(intersection_movement_state->phase_state.value);
      sg->setColor(color.at(0), color.at(1), color.at(2), color.at(3));
    }
    else {
      sg->setColor(color_grey);
    }

    Ogre::Vector3 p;
    p.x = lane.nodes.front().x;
    p.y = lane.nodes.front().y;
    p.z = lane.nodes.front().z;
    sg->setPosition(p);
    signal_groups_.push_back(sg);
  }
}

void MAPEMDisplay::RenderSpatemTexts(Ogre::SceneNode *child_scene_node, IntersectionLane& lane, IntersectionMovementState* intersection_movement_state) {
  std::string text_content;

  if (intersection_movement_state != nullptr) {
    if (intersection_movement_state->time_change_details != nullptr) {
      etsi_its_spatem_ts_msgs::msg::TimeChangeDetails::SharedPtr time_change_details = intersection_movement_state->time_change_details;
      std_msgs::msg::Header& header = intersection_movement_state->header;
      
      if (show_spatem_start_time->getBool()) {
        text_content = "Start time: " 
          + (time_change_details->start_time_is_present 
            ? etsi_its_spatem_ts_msgs::access::parseTimeMarkValueToString(time_change_details->start_time.value, header.stamp.sec, header.stamp.nanosec)
            : "-") 
          + "\n";
      }

      // 'Min end time' is the only required field 
      if (show_spatem_min_end_time->getBool()) {
        text_content += "Min end time: " 
        + etsi_its_spatem_ts_msgs::access::parseTimeMarkValueToString(time_change_details->min_end_time.value, header.stamp.sec, header.stamp.nanosec)
        + "\n";
      }
              
      if (show_spatem_max_end_time->getBool()) {
        text_content += "Max end time: "
          + (time_change_details->max_end_time_is_present 
            ? etsi_its_spatem_ts_msgs::access::parseTimeMarkValueToString(time_change_details->max_end_time.value, header.stamp.sec, header.stamp.nanosec)
            : "-") 
          + "\n";
      }
      if (show_spatem_likely_time->getBool()) {
        text_content += "Likely time: "
          + (time_change_details->likely_time_is_present 
            ? etsi_its_spatem_ts_msgs::access::parseTimeMarkValueToString(time_change_details->likely_time.value, header.stamp.sec, header.stamp.nanosec)
            : "-") 
          + "\n";
      }
      if (show_spatem_confidence->getBool()) {
        text_content += "Confidence: "
          + (time_change_details->confidence_is_present 
            ? std::to_string((int)(etsi_its_spatem_ts_msgs::access::interpretTimeIntervalConfidenceAsFloat(time_change_details->confidence.value) * 100)) + "%" 
            : "-") 
          + "\n";
      }
      if (show_spatem_next_time->getBool()) {
        text_content += "Next time: "
          + (time_change_details->next_time_is_present 
          ? etsi_its_spatem_ts_msgs::access::parseTimeMarkValueToString(time_change_details->next_time.value, header.stamp.sec, header.stamp.nanosec)
          : "-");
      }
    } else {
      text_content = "no time info";
    }
  } else {
    text_content = "-";
  }

  std::shared_ptr<rviz_rendering::MovableText> text_render = std::make_shared<rviz_rendering::MovableText>(text_content, "Liberation Sans", char_height_spatem_->getFloat());
  Ogre::Vector3 halfSize = text_render->getBoundingBox().getHalfSize();
  Ogre::Vector3 offset(
  offset.x = lane.nodes.front().x - halfSize.x * 0.5,
  offset.y = lane.nodes.front().y + halfSize.y,
  offset.z = lane.nodes.front().z + 2);

  text_render->setGlobalTranslation(offset);
  Ogre::ColourValue text_color = rviz_common::properties::qtToOgre(text_color_property_spatem_->getColor());
  text_render->setColor(text_color);        
  child_scene_node->attachObject(text_render.get());
  texts_.push_back(text_render);
}

void MAPEMDisplay::update(float, float) {
  // Check for outdated intersections and movement states
  rclcpp::Time now = rviz_node_->now();
  for (auto it = intersections_.begin(); it != intersections_.end(); ) {
        if (it->second.getAge(now) > mapem_timeout_->getFloat()) it = intersections_.erase(it);
        else {
          it->second.removeOutdatedMovemenStates(now, spatem_timeout_->getFloat());
          ++it;
        }
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

    // visualize intersection
    if(viz_mapem_->getBool()) {
      RenderMapemShapes(child_scene_node);
    }

    // visualize the lanes
    for(size_t i = 0; i<intsctn.lanes.size(); i++) {
      if(viz_mapem_->getBool()) {
        RenderMapemShapesLane(child_scene_node, intsctn.lanes[i]);
      }

      // Signal Groups
      if(viz_spatem_->getBool() && intsctn.lanes[i].signal_group_ids.size() && intsctn.lanes[i].direction != LaneDirection::egress) {

        // Check if SignalGroup is present in IntersectionMovementState of Intersection
        std::unordered_map<int, IntersectionMovementState>::iterator mvmnt_it;
        IntersectionMovementState* mvmnt_ptr = nullptr;

        for(size_t j=0; j<intsctn.lanes[i].signal_group_ids.size(); j++) {
          mvmnt_it = intsctn.movement_states.find(intsctn.lanes[i].signal_group_ids[j]);
          if (mvmnt_it != intsctn.movement_states.end())
          {
            mvmnt_ptr = &mvmnt_it->second;
            break;
          }
        }

        // Visualize current traffic state
        RenderSpatemShapes(child_scene_node, intsctn.lanes[i], mvmnt_ptr);

        // create graphical text to display time information for signal change (see TimeChangeDetail Etsi definition)
        RenderSpatemTexts(child_scene_node, intsctn.lanes[i], mvmnt_ptr);
      }

    }
    // Visualize MAPEM meta-information as text
    if(viz_mapem_->getBool() && show_meta_mapem_->getBool()) {
      RenderMapemTexts(child_scene_node, intsctn);
    }
  }
}

}  // namespace displays
}  // namespace etsi_its_msgs

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(etsi_its_msgs::displays::MAPEMDisplay, rviz_common::Display)