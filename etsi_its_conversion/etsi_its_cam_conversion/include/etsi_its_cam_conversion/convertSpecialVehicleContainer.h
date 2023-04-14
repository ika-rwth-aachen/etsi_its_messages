#pragma once

#include <etsi_its_cam_coding/SpecialVehicleContainer.h>
#include <etsi_its_cam_msgs/SpecialVehicleContainer.h>
#include <etsi_its_cam_conversion/convertPublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertSpecialTransportContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertRoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRescueContainer.h>
#include <etsi_its_cam_conversion/convertEmergencyContainer.h>
#include <etsi_its_cam_conversion/convertSafetyCarContainer.h>

namespace etsi_its_cam_conversion
{
	void convert_SpecialVehicleContainertoRos(const SpecialVehicleContainer_t& _SpecialVehicleContainer_in, etsi_its_cam_msgs::SpecialVehicleContainer& _SpecialVehicleContainer_out)
	{
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer)
		{
			convert_PublicTransportContainertoRos(_SpecialVehicleContainer_in.choice.publicTransportContainer, _SpecialVehicleContainer_out.publicTransportContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer)
		{
			convert_SpecialTransportContainertoRos(_SpecialVehicleContainer_in.choice.specialTransportContainer, _SpecialVehicleContainer_out.specialTransportContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer)
		{
			convert_DangerousGoodsContainertoRos(_SpecialVehicleContainer_in.choice.dangerousGoodsContainer, _SpecialVehicleContainer_out.dangerousGoodsContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic)
		{
			convert_RoadWorksContainerBasictoRos(_SpecialVehicleContainer_in.choice.roadWorksContainerBasic, _SpecialVehicleContainer_out.roadWorksContainerBasic);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer)
		{
			convert_RescueContainertoRos(_SpecialVehicleContainer_in.choice.rescueContainer, _SpecialVehicleContainer_out.rescueContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer)
		{
			convert_EmergencyContainertoRos(_SpecialVehicleContainer_in.choice.emergencyContainer, _SpecialVehicleContainer_out.emergencyContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER;
		}
		if(_SpecialVehicleContainer_in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer)
		{
			convert_SafetyCarContainertoRos(_SpecialVehicleContainer_in.choice.safetyCarContainer, _SpecialVehicleContainer_out.safetyCarContainer);
			_SpecialVehicleContainer_out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER;
		}
	}
	void convert_SpecialVehicleContainertoC(const etsi_its_cam_msgs::SpecialVehicleContainer& _SpecialVehicleContainer_in, SpecialVehicleContainer_t& _SpecialVehicleContainer_out)
	{
		memset(&_SpecialVehicleContainer_out, 0, sizeof(SpecialVehicleContainer_t));
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER)
		{
			convert_PublicTransportContainertoC(_SpecialVehicleContainer_in.publicTransportContainer, _SpecialVehicleContainer_out.choice.publicTransportContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER)
		{
			convert_SpecialTransportContainertoC(_SpecialVehicleContainer_in.specialTransportContainer, _SpecialVehicleContainer_out.choice.specialTransportContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER)
		{
			convert_DangerousGoodsContainertoC(_SpecialVehicleContainer_in.dangerousGoodsContainer, _SpecialVehicleContainer_out.choice.dangerousGoodsContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC)
		{
			convert_RoadWorksContainerBasictoC(_SpecialVehicleContainer_in.roadWorksContainerBasic, _SpecialVehicleContainer_out.choice.roadWorksContainerBasic);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER)
		{
			convert_RescueContainertoC(_SpecialVehicleContainer_in.rescueContainer, _SpecialVehicleContainer_out.choice.rescueContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER)
		{
			convert_EmergencyContainertoC(_SpecialVehicleContainer_in.emergencyContainer, _SpecialVehicleContainer_out.choice.emergencyContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer;
		}
		if(_SpecialVehicleContainer_in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER)
		{
			convert_SafetyCarContainertoC(_SpecialVehicleContainer_in.safetyCarContainer, _SpecialVehicleContainer_out.choice.safetyCarContainer);
			_SpecialVehicleContainer_out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer;
		}
	}
}