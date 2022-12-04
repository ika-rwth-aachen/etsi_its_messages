#pragma once

#include <SpecialVehicleContainer.h>
#include <etsi_its_cam_msgs/SpecialVehicleContainer.h>
#include <convertPublicTransportContainer.h>
#include <convertSpecialTransportContainer.h>
#include <convertDangerousGoodsContainer.h>
#include <convertRoadWorksContainerBasic.h>
#include <convertRescueContainer.h>
#include <convertEmergencyContainer.h>
#include <convertSafetyCarContainer.h>

namespace etsi_its_cam_conversion
{
	etsi_its_cam_msgs::SpecialVehicleContainer convert_SpecialVehicleContainertoRos(const SpecialVehicleContainer_t& _SpecialVehicleContainer_in)
	{
		etsi_its_cam_msgs::SpecialVehicleContainer SpecialVehicleContainer_out;
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer)
		{
			SpecialVehicleContainer_out.publicTransportContainer = convert_PublicTransportContainertoRos(_SpecialVehicleContainer_in.choice.publicTransportContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer)
		{
			SpecialVehicleContainer_out.specialTransportContainer = convert_SpecialTransportContainertoRos(_SpecialVehicleContainer_in.choice.specialTransportContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer)
		{
			SpecialVehicleContainer_out.dangerousGoodsContainer = convert_DangerousGoodsContainertoRos(_SpecialVehicleContainer_in.choice.dangerousGoodsContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic)
		{
			SpecialVehicleContainer_out.roadWorksContainerBasic = convert_RoadWorksContainerBasictoRos(_SpecialVehicleContainer_in.choice.roadWorksContainerBasic);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer)
		{
			SpecialVehicleContainer_out.rescueContainer = convert_RescueContainertoRos(_SpecialVehicleContainer_in.choice.rescueContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer)
		{
			SpecialVehicleContainer_out.emergencyContainer = convert_EmergencyContainertoRos(_SpecialVehicleContainer_in.choice.emergencyContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer)
		{
			SpecialVehicleContainer_out.safetyCarContainer = convert_SafetyCarContainertoRos(_SpecialVehicleContainer_in.choice.safetyCarContainer);
			SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER;
		}
		return SpecialVehicleContainer_out;
	}
}