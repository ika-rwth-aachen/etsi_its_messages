#pragma once

#include <etsi_its_cam_coding/DangerousGoodsContainer.h>
#include <etsi_its_cam_msgs/DangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsBasic.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::DangerousGoodsContainer convert_DangerousGoodsContainertoRos(const DangerousGoodsContainer_t& _DangerousGoodsContainer_in)
	{
		etsi_its_cam_msgs::DangerousGoodsContainer DangerousGoodsContainer_out;
		DangerousGoodsContainer_out.dangerousGoodsBasic = convert_DangerousGoodsBasictoRos(_DangerousGoodsContainer_in.dangerousGoodsBasic);
		return DangerousGoodsContainer_out;
	}
	DangerousGoodsContainer_t convert_DangerousGoodsContainertoC(const etsi_its_cam_msgs::DangerousGoodsContainer& _DangerousGoodsContainer_in)
	{
		DangerousGoodsContainer_t DangerousGoodsContainer_out;
		memset(&DangerousGoodsContainer_out, 0, sizeof(DangerousGoodsContainer_t));
		DangerousGoodsContainer_out.dangerousGoodsBasic = convert_DangerousGoodsBasictoC(_DangerousGoodsContainer_in.dangerousGoodsBasic);
		return DangerousGoodsContainer_out;
	}
}