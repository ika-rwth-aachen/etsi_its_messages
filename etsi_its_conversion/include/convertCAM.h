#pragma once

#include <CAM.h>
#include <etsi_its_cam_msgs/CAM.h>
#include <convertItsPduHeader.h>
#include <convertCoopAwareness.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::CAM convert_CAMtoRos(const CAM_t& _CAM_in)
	{
		etsi_its_cam_msgs::CAM CAM_out;
		CAM_out.header = convert_ItsPduHeadertoRos(_CAM_in.header);
		CAM_out.cam = convert_CoopAwarenesstoRos(_CAM_in.cam);
		return CAM_out;
	}
}