/** ============================================================================
MIT License

Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University

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

#include "etsi_its_cpm_ts_msgs/msg/collective_perception_message.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <etsi_its_msgs_utils/cpm_ts_access.hpp>

#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class CPMRenderObject
 * @brief This class is used to render a CPM object in RViz
 */
class CPMRenderObject
{
  public:
    /**
     * @brief Construct a new CPMRenderObject object from a CPM message
     *
     * @param cpm
     */
    CPMRenderObject(const etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage cpm);

    /**
     * @brief This function validates all float variables that are part of a CPMRenderObject
     *
     */
    bool validateFloats();

    /**
     * @brief Get age of CPM-object
     *
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value
     */
    double getAge(const rclcpp::Time now);

    std_msgs::msg::Header getHeader();
    uint32_t getStationID();
    geometry_msgs::msg::PointStamped getReferencePosition();

    uint8_t getNumberOfObjects();
    geometry_msgs::msg::Pose getPoseOfObject(const uint8_t idx);
    geometry_msgs::msg::Vector3 getDimensionsOfObject(const uint8_t idx);
    geometry_msgs::msg::Vector3 getVelocityOfObject(const uint8_t idx);

    struct Object {
      geometry_msgs::msg::Pose pose;
      geometry_msgs::msg::Vector3 dimensions;
      geometry_msgs::msg::Vector3 velocity;
    };

  private:
    std_msgs::msg::Header header_;
    uint32_t station_id_;
    geometry_msgs::msg::PointStamped reference_position_;
    std::vector<Object> objects_;
};

}  // namespace displays
}  // namespace etsi_its_msgs