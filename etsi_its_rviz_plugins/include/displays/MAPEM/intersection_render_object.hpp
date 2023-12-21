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

#include "etsi_its_mapem_msgs/msg/mapem.hpp"
#include <std_msgs/msg/header.hpp>

#include <etsi_its_msgs_utils/mapem_access.hpp>

#include <rclcpp/rclcpp.hpp>

#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class IntersectionRenderObject
 * @brief
 */
class IntersectionRenderObject
{
  public:
    IntersectionRenderObject(etsi_its_mapem_msgs::msg::IntersectionGeometry intersection, etsi_its_mapem_msgs::msg::MinuteOfTheYear mapem_stamp, rclcpp::Time receive_time);

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
     * @brief Get the IntersectionID
     * 
     * @return unsigned int intersection_id
     */
    unsigned int getIntersectionID();

  private:
    // member variables
    std_msgs::msg::Header header;
    unsigned int intersection_id;


};

}  // namespace displays
}  // namespace etsi_its_msgs