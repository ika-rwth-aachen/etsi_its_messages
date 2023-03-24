#pragma once

#include <etsi_its_cam_coding/ClosedLanes.h>
#include <etsi_its_cam_msgs/ClosedLanes.h>
#include <etsi_its_cam_conversion/convertHardShoulderStatus.h>
#include <etsi_its_cam_conversion/convertDrivingLaneStatus.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::ClosedLanes convert_ClosedLanestoRos(const ClosedLanes_t& _ClosedLanes_in)
	{
		etsi_its_cam_msgs::ClosedLanes ClosedLanes_out;
		if(_ClosedLanes_in.innerhardShoulderStatus)
		{
			ClosedLanes_out.innerhardShoulderStatus = convert_HardShoulderStatustoRos(*_ClosedLanes_in.innerhardShoulderStatus);
			ClosedLanes_out.innerhardShoulderStatus_isPresent = true;
		}
		if(_ClosedLanes_in.outerhardShoulderStatus)
		{
			ClosedLanes_out.outerhardShoulderStatus = convert_HardShoulderStatustoRos(*_ClosedLanes_in.outerhardShoulderStatus);
			ClosedLanes_out.outerhardShoulderStatus_isPresent = true;
		}
		if(_ClosedLanes_in.drivingLaneStatus)
		{
			ClosedLanes_out.drivingLaneStatus = convert_DrivingLaneStatustoRos(*_ClosedLanes_in.drivingLaneStatus);
			ClosedLanes_out.drivingLaneStatus_isPresent = true;
		}
		return ClosedLanes_out;
	}
}