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

#include "displays/MAPEM/intersection_render_object.hpp"
#include <etsi_its_msgs_utils/mapem_ts_access.hpp>

namespace etsi_its_msgs
{
namespace displays
{

  IntersectionRenderObject::IntersectionRenderObject(etsi_its_mapem_ts_msgs::msg::IntersectionGeometry intersection, bool timestamp_is_present, etsi_its_mapem_ts_msgs::msg::MinuteOfTheYear mapem_stamp, rclcpp::Time receive_time) {

    intersection_id = etsi_its_mapem_ts_msgs::access::getIntersectionID(intersection);

    int zone;
    bool northp;
    ref_point = etsi_its_mapem_ts_msgs::access::getRefPointUTMPositionWithConvergenceAngle(intersection, zone, northp, grid_convergence_angle);

    if(timestamp_is_present) {
      uint64_t nanosecs = etsi_its_mapem_ts_msgs::access::getUnixNanosecondsFromMinuteOfTheYear(mapem_stamp, receive_time.nanoseconds());
      header.stamp = rclcpp::Time(nanosecs);
    }
    else {
      header.stamp = receive_time;
    }
    header.frame_id = ref_point.header.frame_id;

    // Parse the lanes
    for(size_t i=0; i<intersection.lane_set.array.size(); i++) {
      etsi_its_mapem_ts_msgs::msg::GenericLane gen_lane = intersection.lane_set.array[i];
      IntersectionLane intsct_lane;
      intsct_lane.lane_id =  gen_lane.lane_id.value;
      // LaneDirection
      std::vector<bool> lane_dir = etsi_its_mapem_ts_msgs::access::getLaneDirection(gen_lane);
      if(lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_INGRESS_PATH] && lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_EGRESS_PATH]) intsct_lane.direction = LaneDirection::bidirectional;
      else if(!lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_INGRESS_PATH] && !lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_EGRESS_PATH]) intsct_lane.direction = LaneDirection::no_travel;
      else if(lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_INGRESS_PATH] && !lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_EGRESS_PATH]) intsct_lane.direction = LaneDirection::ingress; 
      else if(!lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_INGRESS_PATH] && lane_dir[etsi_its_mapem_ts_msgs::msg::LaneDirection::BIT_INDEX_EGRESS_PATH]) intsct_lane.direction = LaneDirection::egress;
      else intsct_lane.direction = LaneDirection::unknown_direction;
      // LaneType
      if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_VEHICLE) intsct_lane.type = LaneType::vehicle;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_CROSSWALK) intsct_lane.type = LaneType::crosswalk;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_BIKE_LANE) intsct_lane.type = LaneType::bike_lane;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_SIDEWALK) intsct_lane.type = LaneType::sidewalk;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_MEDIAN) intsct_lane.type = LaneType::median;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_STRIPING) intsct_lane.type = LaneType::striping;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_TRACKED_VEHICLE) intsct_lane.type = LaneType::tracked_vehicle;
      else if(gen_lane.lane_attributes.lane_type.choice == etsi_its_mapem_ts_msgs::msg::LaneTypeAttributes::CHOICE_PARKING) intsct_lane.type = LaneType::parking;
      else intsct_lane.type = LaneType::unknown_type;
      // Nodes
      if(gen_lane.node_list.choice == etsi_its_mapem_ts_msgs::msg::NodeListXY::CHOICE_NODES) {
        etsi_its_mapem_ts_msgs::msg::NodeSetXY node_set = gen_lane.node_list.nodes;
        for(size_t j=0; j<node_set.array.size(); j++) {
          geometry_msgs::msg::Point p;
          switch (node_set.array[j].delta.choice) {
              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY1:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy1);
                  break;

              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY2:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy2);
                  break;

              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY3:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy3);
                  break;                        
                  
              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY4:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy4);
                  break;                        
              
              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY5:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy5);
                  break;                        
                  
              case etsi_its_mapem_ts_msgs::msg::NodeOffsetPointXY::CHOICE_NODE_XY6:
                  p = etsi_its_mapem_ts_msgs::access::getPointFromNodeXY(node_set.array[j].delta.node_xy6);
                  break;

              default:
                  break;
          }

          if(intsct_lane.nodes.size()) {
            geometry_msgs::msg::Point p_back = intsct_lane.nodes.back();
            p.x += p_back.x;
            p.y += p_back.y;
            p.z += p_back.z;
          }
          intsct_lane.nodes.push_back(p);
        }
      }
      // Signal Groups
      if(gen_lane.connects_to_is_present) {
        for(size_t i=0; i<gen_lane.connects_to.array.size(); i++) {
          if(gen_lane.connects_to.array[i].signal_group_is_present) {
            intsct_lane.signal_group_ids.push_back(gen_lane.connects_to.array[i].signal_group.value);
          }
        }
      }
      // Store lane in vector
      lanes.push_back(intsct_lane);
    }

  }

  bool IntersectionRenderObject::validateFloats() {
    bool valid = true;
    valid = valid && rviz_common::validateFloats(ref_point);
    return valid;
  }

  double IntersectionRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

  void IntersectionRenderObject::removeOutdatedMovemenStates(rclcpp::Time now, double timeout) {
    for (auto it = movement_states.begin(); it != movement_states.end(); ) {
      if ((now-it->second.header.stamp).seconds() > timeout) it = movement_states.erase(it);
      else ++it;
    }
  }

  unsigned int IntersectionRenderObject::getIntersectionID() {
    return intersection_id;
  }

  std_msgs::msg::Header IntersectionRenderObject::getHeader() {
    return header;
  }

  geometry_msgs::msg::Point IntersectionRenderObject::getRefPosition() {
    return ref_point.point;
  }

  tf2::Quaternion IntersectionRenderObject::getGridConvergenceQuaternion() {
    tf2::Quaternion q;
    // yaw offset due to grid-convergence
    q.setRPY(0, 0, grid_convergence_angle * M_PI / 180.0);
    return q;
  }


}  // namespace displays
}  // namespace etsi_its_msgs