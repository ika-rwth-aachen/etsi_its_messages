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

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_public_transport_container) {
    toRos_PublicTransportContainer(in.choice.public_transport_container, out.public_transport_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_special_transport_container) {
    toRos_SpecialTransportContainer(in.choice.special_transport_container, out.special_transport_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerous_goods_container) {
    toRos_DangerousGoodsContainer(in.choice.dangerous_goods_container, out.dangerous_goods_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_road_works_container_basic) {
    toRos_RoadWorksContainerBasic(in.choice.road_works_container_basic, out.road_works_container_basic);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescue_container) {
    toRos_RescueContainer(in.choice.rescue_container, out.rescue_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergency_container) {
    toRos_EmergencyContainer(in.choice.emergency_container, out.emergency_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER;
  }

  if (in.present == SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safety_car_container) {
    toRos_SafetyCarContainer(in.choice.safety_car_container, out.safety_car_container);
    out.choice = etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER;
  }
}

void toStruct_SpecialVehicleContainer(const etsi_its_cam_msgs::SpecialVehicleContainer& in, SpecialVehicleContainer_t& out) {
    
  memset(&out, 0, sizeof(SpecialVehicleContainer_t));

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_PUBLIC_TRANSPORT_CONTAINER) {
    toStruct_PublicTransportContainer(in.public_transport_container, out.choice.public_transport_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_public_transport_container;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SPECIAL_TRANSPORT_CONTAINER) {
    toStruct_SpecialTransportContainer(in.special_transport_container, out.choice.special_transport_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_special_transport_container;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_DANGEROUS_GOODS_CONTAINER) {
    toStruct_DangerousGoodsContainer(in.dangerous_goods_container, out.choice.dangerous_goods_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_dangerous_goods_container;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_ROAD_WORKS_CONTAINER_BASIC) {
    toStruct_RoadWorksContainerBasic(in.road_works_container_basic, out.choice.road_works_container_basic);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_road_works_container_basic;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_RESCUE_CONTAINER) {
    toStruct_RescueContainer(in.rescue_container, out.choice.rescue_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_rescue_container;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_EMERGENCY_CONTAINER) {
    toStruct_EmergencyContainer(in.emergency_container, out.choice.emergency_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_emergency_container;
  }

  if (in.choice == etsi_its_cam_msgs::SpecialVehicleContainer::CHOICE_SAFETY_CAR_CONTAINER) {
    toStruct_SafetyCarContainer(in.safety_car_container, out.choice.safety_car_container);
    out.present = SpecialVehicleContainer_PR::SpecialVehicleContainer_PR_safety_car_container;
  }

}

}