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

#include "etsi_its_mapem_ts_msgs/msg/mapem.hpp"
#include "etsi_its_spatem_ts_msgs/msg/spatem.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

enum LaneDirection {ingress, egress, bidirectional, no_travel, unknown_direction};
enum LaneType {vehicle, crosswalk, bike_lane, sidewalk, median, striping, tracked_vehicle, parking, unknown_type};

typedef struct IntersectionLane {
  uint8_t lane_id;
  LaneType type = LaneType::unknown_type;
  LaneDirection direction = LaneDirection::unknown_direction;
  std::vector<geometry_msgs::msg::Point> nodes; // relative to ref_point of intersection
  std::vector<uint8_t> signal_group_ids;
} IntersectionLane;

typedef struct IntersectionMovementState {
  std_msgs::msg::Header header;
  uint8_t signal_group_id;
  etsi_its_spatem_ts_msgs::msg::MovementPhaseState phase_state;
  etsi_its_spatem_ts_msgs::msg::TimeChangeDetails::SharedPtr time_change_details;

} IntersectionMovementState;

/**
 * @class IntersectionRenderObject
 * @brief
 */
class IntersectionRenderObject
{
  public:
    IntersectionRenderObject(etsi_its_mapem_ts_msgs::msg::IntersectionGeometry intersection, bool timestamp_is_present, etsi_its_mapem_ts_msgs::msg::MinuteOfTheYear mapem_stamp, rclcpp::Time receive_time);

    /**
     * @brief This function validates all float variables that are part of a IntersectionRenderObject
     *
     */
    bool validateFloats();

    /**
     * @brief Get age of corresponding MAPEM
     *
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value
     */
    double getAge(rclcpp::Time now);

    /**
     * @brief Remove outdated MovementStates
     *
     * @param now reference point in time to calculate the age with
     * @param timeout age threshold for that MovementStates should be removed
     */
    void removeOutdatedMovemenStates(rclcpp::Time now, double timeout);

    /**
     * @brief Get the IntersectionID
     * 
     * @return unsigned int intersection_id
     */
    unsigned int getIntersectionID();

    /**
     * @brief Get the header
     * 
     * @return std_msgs::msg::Header 
     */
    std_msgs::msg::Header getHeader();

    /**
     * @brief Get the ref_position object
     * 
     * @return geometry_msgs::msg::Point 
     */
    geometry_msgs::msg::Point getRefPosition();

    /**
     * @brief Return a tf2::Quaternion describing the rotation offset between true-north and grid-north in the UTM zone
     * 
     * @return tf2::Quaternion 
     */
    tf2::Quaternion getGridConvergenceQuaternion();

    // public member variables
    std::vector<IntersectionLane> lanes;
    std::unordered_map<int, IntersectionMovementState> movement_states;

  private:
    // member variables
    std_msgs::msg::Header header;
    unsigned int intersection_id;
    std::vector<unsigned int> layer_ids;
    geometry_msgs::msg::PointStamped ref_point;
    double grid_convergence_angle;    
};

}  // namespace displays
}  // namespace etsi_its_msgs