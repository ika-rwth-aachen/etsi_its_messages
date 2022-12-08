#pragma once

#include <PathPoint.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <convertDeltaReferencePosition.h>
#include <convertPathDeltaTime.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PathPoint convert_PathPointtoRos(const PathPoint_t& _PathPoint_in)
	{
		etsi_its_cam_msgs::PathPoint PathPoint_out;
		PathPoint_out.pathPosition = convert_DeltaReferencePositiontoRos(_PathPoint_in.pathPosition);
		if(_PathPoint_in.pathDeltaTime)
		{
			PathPoint_out.pathDeltaTime = convert_PathDeltaTimetoRos(*_PathPoint_in.pathDeltaTime);
			PathPoint_out.pathDeltaTime_isPresent = true;
		}
		return PathPoint_out;
	}
}