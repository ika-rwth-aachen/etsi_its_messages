// SPDX-License-Identifier: MIT
// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

#include "etsi_its_cam_msgs/msg/cam.hpp"
#include "etsi_its_cam_ts_msgs/msg/cam.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <etsi_its_msgs_utils/cam_access.hpp>
#include <etsi_its_msgs_utils/cam_ts_access.hpp>

#include "rviz_common/validate_floats.hpp"
#include <variant>
#include <cmath>

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
    CAMRenderObject(const std::variant<
                      etsi_its_cam_msgs::msg::CAM,
                      etsi_its_cam_ts_msgs::msg::CAM
                    > & cam_variant,
                    rclcpp::Time receive_time,
                    uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second);

    CAMRenderObject(const etsi_its_cam_msgs::msg::CAM & cam, rclcpp::Time receive_time, uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second)
      : CAMRenderObject(std::variant<etsi_its_cam_msgs::msg::CAM, etsi_its_cam_ts_msgs::msg::CAM>(cam), receive_time, n_leap_seconds) {}

    CAMRenderObject(const etsi_its_cam_ts_msgs::msg::CAM & cam, rclcpp::Time receive_time, uint16_t n_leap_seconds = etsi_its_msgs::LEAP_SECOND_INSERTIONS_SINCE_2004.rbegin()->second)
      : CAMRenderObject(std::variant<etsi_its_cam_msgs::msg::CAM, etsi_its_cam_ts_msgs::msg::CAM>(cam), receive_time, n_leap_seconds) {}

    /**
     * @brief This function validates all float variables that are part of a CAMRenderObject
     *
     */
    bool validateFloats();

    /**
     * @brief Get age of CAM-object
     *
     * @param now reference point in time to calculate the age with
     * @return age in seconds as double value
     */
    double getAge(rclcpp::Time now);

    /**
     * @brief Get header of CAM-object
     *
     * @return std_msgs::msg::Header
     */
    std_msgs::msg::Header getHeader();

    /**
     * @brief Get the StationID of CAM-object
     *
     * @return int
     */
    uint32_t getStationID();

    /**
     * @brief Get the StationType of CAM-object
     *
     * @return int
     */
    int getStationType();

    /**
     * @brief Get pose of CAM-object
     *
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose getPose();

    /**
     * @brief Get dimensions of CAM-Object
     *
     * @return geometry_msgs::msg::Vector3 (x equals length, y equals width, z equals height)
     */
    geometry_msgs::msg::Vector3 getDimensions();

    /**
     * @brief Get speed of CAM-object
     *
     * @return double
     */
    double getSpeed();

    /**
     * @brief Check if the CAM object is TS (release 2)
     *
     * @return true if CAM object is TS, false otherwise
     */
    bool isTS();

  private:
    // member variables
    std_msgs::msg::Header header;
    uint32_t station_id;
    int station_type;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Vector3 dimensions;
    double speed;
    bool is_ts = false;
};

}  // namespace displays
}  // namespace etsi_its_msgs