#pragma once

#include <etsi_its_cam_coding/DangerousGoodsBasic.h>
#include <etsi_its_cam_msgs/DangerousGoodsBasic.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DangerousGoodsBasic convert_DangerousGoodsBasictoRos(const DangerousGoodsBasic_t& _DangerousGoodsBasic_in)
	{
		etsi_its_cam_msgs::DangerousGoodsBasic DangerousGoodsBasic_out;
		DangerousGoodsBasic_out.value = _DangerousGoodsBasic_in;
		return DangerousGoodsBasic_out;
	}
	DangerousGoodsBasic_t convert_DangerousGoodsBasictoC(const etsi_its_cam_msgs::DangerousGoodsBasic& _DangerousGoodsBasic_in)
	{
		DangerousGoodsBasic_t DangerousGoodsBasic_out;
		DangerousGoodsBasic_out = _DangerousGoodsBasic_in.value;
		return DangerousGoodsBasic_out;
	}
}