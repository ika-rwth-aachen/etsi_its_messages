#pragma once

#include <etsi_its_cam_coding/DeltaLatitude.h>
#include <etsi_its_cam_msgs/DeltaLatitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion
{
	void convert_DeltaLatitudetoRos(const DeltaLatitude_t& _DeltaLatitude_in, etsi_its_cam_msgs::DeltaLatitude& _DeltaLatitude_out)
	{
		convert_toRos(_DeltaLatitude_in, _DeltaLatitude_out.value);
	}
	void convert_DeltaLatitudetoC(const etsi_its_cam_msgs::DeltaLatitude& _DeltaLatitude_in, DeltaLatitude_t& _DeltaLatitude_out)
	{
		memset(&_DeltaLatitude_out, 0, sizeof(DeltaLatitude_t));
		convert_toC(_DeltaLatitude_in.value, _DeltaLatitude_out);
	}
}