#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_msgs/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>

namespace etsi_its_cam_conversion
{
	void convert_CurvaturetoRos(const Curvature_t& _Curvature_in, etsi_its_cam_msgs::Curvature& _Curvature_out)
	{
		convert_CurvatureValuetoRos(_Curvature_in.curvatureValue, _Curvature_out.curvatureValue);
		convert_CurvatureConfidencetoRos(_Curvature_in.curvatureConfidence, _Curvature_out.curvatureConfidence);
	}
	void convert_CurvaturetoC(const etsi_its_cam_msgs::Curvature& _Curvature_in, Curvature_t& _Curvature_out)
	{
		memset(&_Curvature_out, 0, sizeof(Curvature_t));
		convert_CurvatureValuetoC(_Curvature_in.curvatureValue, _Curvature_out.curvatureValue);
		convert_CurvatureConfidencetoC(_Curvature_in.curvatureConfidence, _Curvature_out.curvatureConfidence);
	}
}