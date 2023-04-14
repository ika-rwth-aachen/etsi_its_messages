#pragma once

#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_conversion/convertDeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertPathDeltaTime.h>

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
	PathPoint_t convert_PathPointtoC(const etsi_its_cam_msgs::PathPoint& _PathPoint_in)
	{
		PathPoint_t PathPoint_out;
		memset(&PathPoint_out, 0, sizeof(PathPoint_t));
		PathPoint_out.pathPosition = convert_DeltaReferencePositiontoC(_PathPoint_in.pathPosition);
		if(_PathPoint_in.pathDeltaTime_isPresent)
		{
			auto pathDeltaTime = convert_PathDeltaTimetoC(_PathPoint_in.pathDeltaTime);
			PathPoint_out.pathDeltaTime = new PathDeltaTime_t(pathDeltaTime);
		}
		return PathPoint_out;
	}
}