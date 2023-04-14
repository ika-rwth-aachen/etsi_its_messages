#pragma once

#include <etsi_its_cam_coding/PosConfidenceEllipse.h>
#include <etsi_its_cam_msgs/PosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>

namespace etsi_its_cam_conversion
{
	void convert_PosConfidenceEllipsetoRos(const PosConfidenceEllipse_t& _PosConfidenceEllipse_in, etsi_its_cam_msgs::PosConfidenceEllipse& _PosConfidenceEllipse_out)
	{
		convert_SemiAxisLengthtoRos(_PosConfidenceEllipse_in.semiMajorConfidence, _PosConfidenceEllipse_out.semiMajorConfidence);
		convert_SemiAxisLengthtoRos(_PosConfidenceEllipse_in.semiMinorConfidence, _PosConfidenceEllipse_out.semiMinorConfidence);
		convert_HeadingValuetoRos(_PosConfidenceEllipse_in.semiMajorOrientation, _PosConfidenceEllipse_out.semiMajorOrientation);
	}
	void convert_PosConfidenceEllipsetoC(const etsi_its_cam_msgs::PosConfidenceEllipse& _PosConfidenceEllipse_in, PosConfidenceEllipse_t& _PosConfidenceEllipse_out)
	{
		memset(&_PosConfidenceEllipse_out, 0, sizeof(PosConfidenceEllipse_t));
		convert_SemiAxisLengthtoC(_PosConfidenceEllipse_in.semiMajorConfidence, _PosConfidenceEllipse_out.semiMajorConfidence);
		convert_SemiAxisLengthtoC(_PosConfidenceEllipse_in.semiMinorConfidence, _PosConfidenceEllipse_out.semiMinorConfidence);
		convert_HeadingValuetoC(_PosConfidenceEllipse_in.semiMajorOrientation, _PosConfidenceEllipse_out.semiMajorOrientation);
	}
}