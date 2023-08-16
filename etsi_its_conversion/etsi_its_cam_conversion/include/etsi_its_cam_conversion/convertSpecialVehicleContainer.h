#pragma once

#include <etsi_its_cam_coding/SpecialVehicleContainer.h>
#include <etsi_its_cam_conversion/convertPublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertSpecialTransportContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertRoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRescueContainer.h>
#include <etsi_its_cam_conversion/convertEmergencyContainer.h>
#include <etsi_its_cam_conversion/convertSafetyCarContainer.h>
#include <etsi_its_cam_msgs/SpecialVehicleContainer.h>


namespace etsi_its_cam_conversion {

void toRos_SpecialVehicleContainer(const SpecialVehicleContainer_t& in, etsi_its_cam_msgs::SpecialVehicleContainer& out) {

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer) {
    toRos_PublicTransportContainer(in.choice.publicTransportContainer, out.publicTransportContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer) {
    toRos_SpecialTransportContainer(in.choice.specialTransportContainer, out.specialTransportContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer) {
    toRos_DangerousGoodsContainer(in.choice.dangerousGoodsContainer, out.dangerousGoodsContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic) {
    toRos_RoadWorksContainerBasic(in.choice.roadWorksContainerBasic, out.roadWorksContainerBasic);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer) {
    toRos_RescueContainer(in.choice.rescueContainer, out.rescueContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer) {
    toRos_EmergencyContainer(in.choice.emergencyContainer, out.emergencyContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer) {
    toRos_SafetyCarContainer(in.choice.safetyCarContainer, out.safetyCarContainer);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER;
  }
}

void toStruct_SpecialVehicleContainer(const etsi_its_cam_msgs::SpecialVehicleContainer& in, SpecialVehicleContainer_t& out) {
    
  memset(&out, 0, sizeof(SpecialVehicleContainer_t));

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER) {
    toStruct_PublicTransportContainer(in.publicTransportContainer, out.choice.publicTransportContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER) {
    toStruct_SpecialTransportContainer(in.specialTransportContainer, out.choice.specialTransportContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER) {
    toStruct_DangerousGoodsContainer(in.dangerousGoodsContainer, out.choice.dangerousGoodsContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC) {
    toStruct_RoadWorksContainerBasic(in.roadWorksContainerBasic, out.choice.roadWorksContainerBasic);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER) {
    toStruct_RescueContainer(in.rescueContainer, out.choice.rescueContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER) {
    toStruct_EmergencyContainer(in.emergencyContainer, out.choice.emergencyContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER) {
    toStruct_SafetyCarContainer(in.safetyCarContainer, out.choice.safetyCarContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer;
  }

}

}