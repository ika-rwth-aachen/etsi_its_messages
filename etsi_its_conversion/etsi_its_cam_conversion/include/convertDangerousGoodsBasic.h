#pragma once

#include <DangerousGoodsBasic.h>
#include <etsi_its_cam_msgs/DangerousGoodsBasic.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DangerousGoodsBasic convert_DangerousGoodsBasictoRos(const DangerousGoodsBasic_t& _DangerousGoodsBasic_in)
	{
		etsi_its_cam_msgs::DangerousGoodsBasic DangerousGoodsBasic_out;
		DangerousGoodsBasic_out.value = _DangerousGoodsBasic_in;
		return DangerousGoodsBasic_out;
	}
}