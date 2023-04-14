#pragma once

#include <etsi_its_cam_coding/DeltaReferencePosition.h>
#include <etsi_its_cam_msgs/DeltaReferencePosition.h>
#include <etsi_its_cam_conversion/convertDeltaLatitude.h>
#include <etsi_its_cam_conversion/convertDeltaLongitude.h>
#include <etsi_its_cam_conversion/convertDeltaAltitude.h>

namespace etsi_its_cam_conversion
{
	void convert_DeltaReferencePositiontoRos(const DeltaReferencePosition_t& _DeltaReferencePosition_in, etsi_its_cam_msgs::DeltaReferencePosition& _DeltaReferencePosition_out)
	{
		convert_DeltaLatitudetoRos(_DeltaReferencePosition_in.deltaLatitude, _DeltaReferencePosition_out.deltaLatitude);
		convert_DeltaLongitudetoRos(_DeltaReferencePosition_in.deltaLongitude, _DeltaReferencePosition_out.deltaLongitude);
		convert_DeltaAltitudetoRos(_DeltaReferencePosition_in.deltaAltitude, _DeltaReferencePosition_out.deltaAltitude);
	}
	void convert_DeltaReferencePositiontoC(const etsi_its_cam_msgs::DeltaReferencePosition& _DeltaReferencePosition_in, DeltaReferencePosition_t& _DeltaReferencePosition_out)
	{
		memset(&_DeltaReferencePosition_out, 0, sizeof(DeltaReferencePosition_t));
		convert_DeltaLatitudetoC(_DeltaReferencePosition_in.deltaLatitude, _DeltaReferencePosition_out.deltaLatitude);
		convert_DeltaLongitudetoC(_DeltaReferencePosition_in.deltaLongitude, _DeltaReferencePosition_out.deltaLongitude);
		convert_DeltaAltitudetoC(_DeltaReferencePosition_in.deltaAltitude, _DeltaReferencePosition_out.deltaAltitude);
	}
}