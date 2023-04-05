#pragma once

#include <etsi_its_cam_coding/PosConfidenceEllipse.h>
#include <etsi_its_cam_msgs/PosConfidenceEllipse.h>
#include <etsi_its_cam_conversion/convertSemiAxisLength.h>
#include <etsi_its_cam_conversion/convertHeadingValue.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::PosConfidenceEllipse convert_PosConfidenceEllipsetoRos(const PosConfidenceEllipse_t& _PosConfidenceEllipse_in)
	{
		etsi_its_cam_msgs::PosConfidenceEllipse PosConfidenceEllipse_out;
		PosConfidenceEllipse_out.semiMajorConfidence = convert_SemiAxisLengthtoRos(_PosConfidenceEllipse_in.semiMajorConfidence);
		PosConfidenceEllipse_out.semiMinorConfidence = convert_SemiAxisLengthtoRos(_PosConfidenceEllipse_in.semiMinorConfidence);
		PosConfidenceEllipse_out.semiMajorOrientation = convert_HeadingValuetoRos(_PosConfidenceEllipse_in.semiMajorOrientation);
		return PosConfidenceEllipse_out;
	}
	PosConfidenceEllipse_t convert_PosConfidenceEllipsetoC(const etsi_its_cam_msgs::PosConfidenceEllipse& _PosConfidenceEllipse_in)
	{
		PosConfidenceEllipse_t PosConfidenceEllipse_out;
		PosConfidenceEllipse_out.semiMajorConfidence = convert_SemiAxisLengthtoC(_PosConfidenceEllipse_in.semiMajorConfidence);
		PosConfidenceEllipse_out.semiMinorConfidence = convert_SemiAxisLengthtoC(_PosConfidenceEllipse_in.semiMinorConfidence);
		PosConfidenceEllipse_out.semiMajorOrientation = convert_HeadingValuetoC(_PosConfidenceEllipse_in.semiMajorOrientation);
		return PosConfidenceEllipse_out;
	}
}