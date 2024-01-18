#include "etsi_its_denm_msgs/msg/denm.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <etsi_its_msgs_utils/denm_access.hpp>

#include "rviz_common/validate_floats.hpp"

namespace etsi_its_msgs
{
namespace displays
{

/**
 * @class DENMRenderObject
 * @brief
 */
class DENMRenderObject
{
  public:
    DENMRenderObject(etsi_its_denm_msgs::msg::DENM denm, rclcpp::Time receive_time, uint16_t n_leap_seconds=etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.end()->second);

    /**
     * @brief This function validates all float variables that are part of a DENMRenderObject
     *
     */
    bool validateFloats();

    /**
     * @brief Get age of DENM-object
     *
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value
     */
    double getAge(rclcpp::Time now);

    /**
     * @brief Get header of DENM-object
     *
     * @return std_msgs::msg::Header
     */
    std_msgs::msg::Header getHeader();

    /**
     * @brief Get the StationID of DENM-object
     *
     * @return int
     */
    int getStationID();

    /**
     * @brief Get the StationType of DENM-object
     *
     * @return int
     */
    int getStationType();

    /**
     * @brief Get pose of DENM-object
     *
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose getPose();

    /**
     * @brief Get dimensions of DENM-Object
     *
     * @return geometry_msgs::msg::Vector3 (x equals length, y equals width, z equals height)
     */
    geometry_msgs::msg::Vector3 getDimensions();

    /**
     * @brief Get speed of DENM-object
     *
     * @return double
     */
    double getSpeed();

    /**
     * @brief Get the Cause Code object
     *
     * @return std::string
     */
    std::string getCauseCode();

    /**
     * @brief Get the Sub Cause Code object
     *
     * @return std::string
     */
    std::string getSubCauseCode();
  private:
    // member variables
    std_msgs::msg::Header header;
    int station_id;
    int station_type;
    std::string cause_code_type;
    std::string sub_cause_code_type;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Vector3 dimensions;
    double speed;

};

}  // namespace displays
}  // namespace etsi_its_msgs