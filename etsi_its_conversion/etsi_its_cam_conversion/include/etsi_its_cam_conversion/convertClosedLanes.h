#pragma once

#include <etsi_its_cam_coding/ClosedLanes.h>
#include <etsi_its_cam_msgs/ClosedLanes.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>

#include <etsi_its_cam_conversion/convertDrivingLaneStatus.h>

namespace etsi_its_cam_conversion {
  
void convert_ClosedLanestoRos(const ClosedLanes_t& _ClosedLanes_in, etsi_its_cam_msgs::ClosedLanes& _ClosedLanes_out) {
  if (_ClosedLanes_in.innerhardShoulderStatus) {
    convert_HardShoulderStatustoRos(*_ClosedLanes_in.innerhardShoulderStatus, _ClosedLanes_out.innerhardShoulderStatus);
    _ClosedLanes_out.innerhardShoulderStatus_isPresent = true;
  }
  if (_ClosedLanes_in.outerhardShoulderStatus) {
    convert_HardShoulderStatustoRos(*_ClosedLanes_in.outerhardShoulderStatus, _ClosedLanes_out.outerhardShoulderStatus);
    _ClosedLanes_out.outerhardShoulderStatus_isPresent = true;
  }
  if (_ClosedLanes_in.drivingLaneStatus) {
    convert_DrivingLaneStatustoRos(*_ClosedLanes_in.drivingLaneStatus, _ClosedLanes_out.drivingLaneStatus);
    _ClosedLanes_out.drivingLaneStatus_isPresent = true;
  }
}

void convert_ClosedLanestoC(const etsi_its_cam_msgs::ClosedLanes& _ClosedLanes_in, ClosedLanes_t& _ClosedLanes_out) {
  memset(&_ClosedLanes_out, 0, sizeof(ClosedLanes_t));
  if (_ClosedLanes_in.innerhardShoulderStatus_isPresent) {
    HardShoulderStatus_t innerhardShoulderStatus;
    convert_HardShoulderStatustoC(_ClosedLanes_in.innerhardShoulderStatus, innerhardShoulderStatus);
    _ClosedLanes_out.innerhardShoulderStatus = new HardShoulderStatus_t(innerhardShoulderStatus);
  }
  if (_ClosedLanes_in.outerhardShoulderStatus_isPresent) {
    HardShoulderStatus_t outerhardShoulderStatus;
    convert_HardShoulderStatustoC(_ClosedLanes_in.outerhardShoulderStatus, outerhardShoulderStatus);
    _ClosedLanes_out.outerhardShoulderStatus = new HardShoulderStatus_t(outerhardShoulderStatus);
  }
  if (_ClosedLanes_in.drivingLaneStatus_isPresent) {
    DrivingLaneStatus_t drivingLaneStatus;
    convert_DrivingLaneStatustoC(_ClosedLanes_in.drivingLaneStatus, drivingLaneStatus);
    _ClosedLanes_out.drivingLaneStatus = new DrivingLaneStatus_t(drivingLaneStatus);
  }
}

}