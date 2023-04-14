#pragma once

#include <etsi_its_cam_coding/PerformanceClass.h>
#include <etsi_its_cam_msgs/PerformanceClass.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PerformanceClass convert_PerformanceClasstoRos(const PerformanceClass_t& _PerformanceClass_in)
	{
		etsi_its_cam_msgs::PerformanceClass PerformanceClass_out;
		convert_toRos(_PerformanceClass_in, PerformanceClass_out.value);
		return PerformanceClass_out;
	}
	PerformanceClass_t convert_PerformanceClasstoC(const etsi_its_cam_msgs::PerformanceClass& _PerformanceClass_in)
	{
		PerformanceClass_t PerformanceClass_out;
		memset(&PerformanceClass_out, 0, sizeof(PerformanceClass_t));
		convert_toC(_PerformanceClass_in.value, PerformanceClass_out);
		return PerformanceClass_out;
	}
}