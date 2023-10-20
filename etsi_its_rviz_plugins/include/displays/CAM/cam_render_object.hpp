#include "etsi_its_cam_msgs/msg/cam.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <etsi_its_msgs/cam_access.hpp>

#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class CAMRenderObject
 * @brief 
 */
class CAMRenderObject
{
  public:
    CAMRenderObject(etsi_its_cam_msgs::msg::CAM cam, rclcpp::Time receive_time, uint16_t n_leap_seconds=etsi_its_msgs::N_LEAP_SECONDS);

    /**
     * @brief Get the age of a CAMRenderObject
     * 
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value 
     */
    double getAge(rclcpp::Time now);

    /**
     * @brief This function validates all float variables that are part of a CAMRenderObject
     * 
     */
    bool validateFloats();

    // Public member variables
    std_msgs::msg::Header header;
    int station_id;
    double width, length, height;
    geometry_msgs::msg::Pose pose;
    double speed;
    
};

}  // namespace displays
}  // namespace etsi_its_msgs