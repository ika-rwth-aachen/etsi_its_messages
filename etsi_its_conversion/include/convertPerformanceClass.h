#pragma once

#include <PerformanceClass.h>
#include <etsi_its_cam_msgs/PerformanceClass.h>
#include <convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PerformanceClass convert_PerformanceClasstoRos(const PerformanceClass_t& _PerformanceClass_in)
	{
		etsi_its_cam_msgs::PerformanceClass PerformanceClass_out;
		convert_toRos(_PerformanceClass_in, PerformanceClass_out.value);
		return PerformanceClass_out;
	}
}