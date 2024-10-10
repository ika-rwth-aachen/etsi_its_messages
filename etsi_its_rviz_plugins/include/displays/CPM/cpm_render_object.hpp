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
 * @brief
 */
class CPMRenderObject
{
  public:
    /**
     * @brief Construct a new CPMRenderObject object
     *
     * @param cpm
     * @param receive_time
     * @param n_leap_seconds
     * @param number_of_object
     */
    CPMRenderObject(etsi_its_cpm_ts_msgs::msg::CollectivePerceptionMessage cpm, uint8_t number_of_object=0);

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
    double getAge(rclcpp::Time now);

    /**
     * @brief Get header of CPM-object
     *
     * @return std_msgs::msg::Header
     */
    std_msgs::msg::Header getHeader();

    /**
     * @brief Get the StationID of CPM-object
     *
     * @return int
     */
    uint32_t getStationID();

    /**
     * @brief Get pose of CPM-object
     *
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose getPose();

    /**
     * @brief Get dimensions of CPM-Object
     *
     * @return geometry_msgs::msg::Vector3 (x equals length, y equals width, z equals height)
     */
    geometry_msgs::msg::Vector3 getDimensions();

    /**
     * @brief Get speed of CPM-object
     *
     * @return double
     */
    geometry_msgs::msg::Vector3 getVelocity();

    /**
     * @brief Get number of objects in CPM-object
     *
     * @return uint8_t
     */
    uint8_t getNumberOfObjects();

  private:
    // member variables
    std_msgs::msg::Header header_;
    uint32_t station_id_;
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::Vector3 dimensions_;
    geometry_msgs::msg::Vector3 velocity_;

};

}  // namespace displays
}  // namespace etsi_its_msgs