#pragma once

#include <etsi_its_cam_coding/ClosedLanes.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>
#include <etsi_its_cam_conversion/convertDrivingLaneStatus.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/closed_lanes.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/ClosedLanes.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_ClosedLanes(const ClosedLanes_t& in, cam_msgs::ClosedLanes& out) {

  if (in.innerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.innerhardShoulderStatus, out.innerhard_shoulder_status);
    out.innerhard_shoulder_status_is_present = true;
  }

  if (in.outerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.outerhardShoulderStatus, out.outerhard_shoulder_status);
    out.outerhard_shoulder_status_is_present = true;
  }

  if (in.drivingLaneStatus) {
    toRos_DrivingLaneStatus(*in.drivingLaneStatus, out.driving_lane_status);
    out.driving_lane_status_is_present = true;
  }

}

void toStruct_ClosedLanes(const cam_msgs::ClosedLanes& in, ClosedLanes_t& out) {
    
  memset(&out, 0, sizeof(ClosedLanes_t));

  if (in.innerhard_shoulder_status_is_present) {
    HardShoulderStatus_t innerhard_shoulder_status;
    toStruct_HardShoulderStatus(in.innerhard_shoulder_status, innerhard_shoulder_status);
    out.innerhardShoulderStatus = new HardShoulderStatus_t(innerhard_shoulder_status);
  }

  if (in.outerhard_shoulder_status_is_present) {
    HardShoulderStatus_t outerhard_shoulder_status;
    toStruct_HardShoulderStatus(in.outerhard_shoulder_status, outerhard_shoulder_status);
    out.outerhardShoulderStatus = new HardShoulderStatus_t(outerhard_shoulder_status);
  }

  if (in.driving_lane_status_is_present) {
    DrivingLaneStatus_t driving_lane_status;
    toStruct_DrivingLaneStatus(in.driving_lane_status, driving_lane_status);
    out.drivingLaneStatus = new DrivingLaneStatus_t(driving_lane_status);
  }

}

}