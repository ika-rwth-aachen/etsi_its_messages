#pragma once

#include <DangerousGoodsContainer.h>
#include <etsi_its_cam_msgs/DangerousGoodsContainer.h>
#include <convertDangerousGoodsBasic.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DangerousGoodsContainer convert_DangerousGoodsContainertoRos(const DangerousGoodsContainer_t& _DangerousGoodsContainer_in)
	{
		etsi_its_cam_msgs::DangerousGoodsContainer DangerousGoodsContainer_out;
		DangerousGoodsContainer_out.dangerousGoodsBasic = convert_DangerousGoodsBasictoRos(_DangerousGoodsContainer_in.dangerousGoodsBasic);
		return DangerousGoodsContainer_out;
	}
}