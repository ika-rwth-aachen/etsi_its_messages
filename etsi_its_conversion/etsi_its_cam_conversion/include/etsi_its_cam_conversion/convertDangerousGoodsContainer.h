#pragma once

#include <etsi_its_cam_coding/DangerousGoodsContainer.h>
#include <etsi_its_cam_msgs/DangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsBasic.h>

namespace etsi_its_cam_conversion
{
	void convert_DangerousGoodsContainertoRos(const DangerousGoodsContainer_t& _DangerousGoodsContainer_in, etsi_its_cam_msgs::DangerousGoodsContainer& _DangerousGoodsContainer_out)
	{
		convert_DangerousGoodsBasictoRos(_DangerousGoodsContainer_in.dangerousGoodsBasic, _DangerousGoodsContainer_out.dangerousGoodsBasic);
	}
	void convert_DangerousGoodsContainertoC(const etsi_its_cam_msgs::DangerousGoodsContainer& _DangerousGoodsContainer_in, DangerousGoodsContainer_t& _DangerousGoodsContainer_out)
	{
		memset(&_DangerousGoodsContainer_out, 0, sizeof(DangerousGoodsContainer_t));
		convert_DangerousGoodsBasictoC(_DangerousGoodsContainer_in.dangerousGoodsBasic, _DangerousGoodsContainer_out.dangerousGoodsBasic);
	}
}