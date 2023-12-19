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

#include "displays/MAPEM/mapem_render_object.hpp"

namespace etsi_its_msgs
{
namespace displays
{

  MAPEMRenderObject::MAPEMRenderObject(etsi_its_mapem_msgs::msg::MAPEM mapem, rclcpp::Time receive_time, uint16_t n_leap_seconds) {

    if(mapem.map.time_stamp_is_present)
    {
      
    }

  }

  bool MAPEMRenderObject::validateFloats() {
    bool valid = true;
    //valid = valid && rviz_common::validateFloats(ref_point);
    return valid;
  }

  double MAPEMRenderObject::getAge(rclcpp::Time now) {
    return (now-header.stamp).seconds();
  }

}  // namespace displays
}  // namespace etsi_its_msgs