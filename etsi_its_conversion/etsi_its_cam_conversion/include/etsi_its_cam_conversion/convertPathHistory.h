#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PathHistory.h>
#include <etsi_its_cam_coding/PathPoint.h>
#include <etsi_its_cam_msgs/PathHistory.h>
#include <etsi_its_cam_msgs/PathPoint.h>
#include <etsi_its_cam_conversion/convertPathPoint.h>

#include <etsi_its_cam_coding/asn_SEQUENCE_OF.h>

namespace etsi_its_cam_conversion
{
	void convert_PathHistorytoRos(const PathHistory_t& _PathHistory_in, etsi_its_cam_msgs::PathHistory& _PathHistory_out)
	{
		for (int i = 0; i < _PathHistory_in.list.count; i++) {
			etsi_its_cam_msgs::PathPoint pathPoint;
			convert_PathPointtoRos(*(_PathHistory_in.list.array[i]), pathPoint);
			_PathHistory_out.array.push_back(pathPoint);
		}
	}
	void convert_PathHistorytoC(const etsi_its_cam_msgs::PathHistory& _PathHistory_in, PathHistory_t& _PathHistory_out)
	{
		memset(&_PathHistory_out, 0, sizeof(PathHistory_t));
		for (int i = 0; i < _PathHistory_in.array.size(); i++) {
			PathPoint_t pathPoint;
			convert_PathPointtoC(_PathHistory_in.array[i], pathPoint);
			PathPoint_t* pathPoint_ptr = new PathPoint_t(pathPoint);
			int status = asn_sequence_add(&_PathHistory_out, pathPoint_ptr);
			if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
		}
	}
}