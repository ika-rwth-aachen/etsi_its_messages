#pragma once

#include <etsi_its_cam_coding/ClosedLanes.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>
#include <etsi_its_cam_conversion/convertDrivingLaneStatus.h>
#include <etsi_its_cam_msgs/ClosedLanes.h>


namespace etsi_its_cam_conversion {

void toRos_ClosedLanes(const ClosedLanes_t& in, etsi_its_cam_msgs::ClosedLanes& out) {

  if (in.innerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.innerhardShoulderStatus, out.innerhardShoulderStatus);
    out.innerhardShoulderStatus_isPresent = true;
  }

  if (in.outerhardShoulderStatus) {
    toRos_HardShoulderStatus(*in.outerhardShoulderStatus, out.outerhardShoulderStatus);
    out.outerhardShoulderStatus_isPresent = true;
  }

  if (in.drivingLaneStatus) {
    toRos_DrivingLaneStatus(*in.drivingLaneStatus, out.drivingLaneStatus);
    out.drivingLaneStatus_isPresent = true;
  }

}

void toStruct_ClosedLanes(const etsi_its_cam_msgs::ClosedLanes& in, ClosedLanes_t& out) {
    
  memset(&out, 0, sizeof(ClosedLanes_t));

  if (in.innerhardShoulderStatus_isPresent) {
    HardShoulderStatus_t innerhardShoulderStatus;
    toStruct_HardShoulderStatus(in.innerhardShoulderStatus, innerhardShoulderStatus);
    out.innerhardShoulderStatus = new HardShoulderStatus_t(innerhardShoulderStatus);
  }

  if (in.outerhardShoulderStatus_isPresent) {
    HardShoulderStatus_t outerhardShoulderStatus;
    toStruct_HardShoulderStatus(in.outerhardShoulderStatus, outerhardShoulderStatus);
    out.outerhardShoulderStatus = new HardShoulderStatus_t(outerhardShoulderStatus);
  }

  if (in.drivingLaneStatus_isPresent) {
    DrivingLaneStatus_t drivingLaneStatus;
    toStruct_DrivingLaneStatus(in.drivingLaneStatus, drivingLaneStatus);
    out.drivingLaneStatus = new DrivingLaneStatus_t(drivingLaneStatus);
  }

}

}