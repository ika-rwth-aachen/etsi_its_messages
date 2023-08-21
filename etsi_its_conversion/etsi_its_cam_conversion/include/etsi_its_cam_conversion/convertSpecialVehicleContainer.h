#pragma once

#include <etsi_its_cam_coding/SpecialVehicleContainer.h>
#include <etsi_its_cam_conversion/convertPublicTransportContainer.h>
#include <etsi_its_cam_conversion/convertSpecialTransportContainer.h>
#include <etsi_its_cam_conversion/convertDangerousGoodsContainer.h>
#include <etsi_its_cam_conversion/convertRoadWorksContainerBasic.h>
#include <etsi_its_cam_conversion/convertRescueContainer.h>
#include <etsi_its_cam_conversion/convertEmergencyContainer.h>
#include <etsi_its_cam_conversion/convertSafetyCarContainer.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/special_vehicle_container.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/SpecialVehicleContainer.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_SpecialVehicleContainer(const SpecialVehicleContainer_t& in, cam_msgs::SpecialVehicleContainer& out) {

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer) {
    toRos_PublicTransportContainer(in.choice.publicTransportContainer, out.public_transport_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer) {
    toRos_SpecialTransportContainer(in.choice.specialTransportContainer, out.special_transport_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer) {
    toRos_DangerousGoodsContainer(in.choice.dangerousGoodsContainer, out.dangerous_goods_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic) {
    toRos_RoadWorksContainerBasic(in.choice.roadWorksContainerBasic, out.road_works_container_basic);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer) {
    toRos_RescueContainer(in.choice.rescueContainer, out.rescue_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer) {
    toRos_EmergencyContainer(in.choice.emergencyContainer, out.emergency_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer) {
    toRos_SafetyCarContainer(in.choice.safetyCarContainer, out.safety_car_container);
    out.choice = cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER;
  }
}

void toStruct_SpecialVehicleContainer(const cam_msgs::SpecialVehicleContainer& in, SpecialVehicleContainer_t& out) {
    
  memset(&out, 0, sizeof(SpecialVehicleContainer_t));

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER) {
    toStruct_PublicTransportContainer(in.public_transport_container, out.choice.publicTransportContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_publicTransportContainer;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER) {
    toStruct_SpecialTransportContainer(in.special_transport_container, out.choice.specialTransportContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_specialTransportContainer;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER) {
    toStruct_DangerousGoodsContainer(in.dangerous_goods_container, out.choice.dangerousGoodsContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerousGoodsContainer;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC) {
    toStruct_RoadWorksContainerBasic(in.road_works_container_basic, out.choice.roadWorksContainerBasic);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_roadWorksContainerBasic;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER) {
    toStruct_RescueContainer(in.rescue_container, out.choice.rescueContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescueContainer;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER) {
    toStruct_EmergencyContainer(in.emergency_container, out.choice.emergencyContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergencyContainer;
  }

  if (in.choice == cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER) {
    toStruct_SafetyCarContainer(in.safety_car_container, out.choice.safetyCarContainer);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safetyCarContainer;
  }

}

}