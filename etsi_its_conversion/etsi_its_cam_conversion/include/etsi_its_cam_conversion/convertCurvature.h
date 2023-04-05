#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_msgs/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::Curvature convert_CurvaturetoRos(const Curvature_t& _Curvature_in)
	{
		etsi_its_cam_msgs::Curvature Curvature_out;
		Curvature_out.curvatureValue = convert_CurvatureValuetoRos(_Curvature_in.curvatureValue);
		Curvature_out.curvatureConfidence = convert_CurvatureConfidencetoRos(_Curvature_in.curvatureConfidence);
		return Curvature_out;
	}
	Curvature_t convert_CurvaturetoC(const etsi_its_cam_msgs::Curvature& _Curvature_in)
	{
		Curvature_t Curvature_out;
		memset(&Curvature_out, 0, sizeof(Curvature_t));
		Curvature_out.curvatureValue = convert_CurvatureValuetoC(_Curvature_in.curvatureValue);
		Curvature_out.curvatureConfidence = convert_CurvatureConfidencetoC(_Curvature_in.curvatureConfidence);
		return Curvature_out;
	}
}