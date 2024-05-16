//// SEQUENCE CoopAwareness


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CoopAwareness.h>
#include <etsi_its_cam_conversion/convertGenerationDeltaTime.h>
#include <etsi_its_cam_conversion/convertCamParameters.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CoopAwareness.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/coop_awareness.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CoopAwareness(const CoopAwareness_t& in, cam_msgs::CoopAwareness& out) {
  toRos_GenerationDeltaTime(in.generationDeltaTime, out.generation_delta_time);
  toRos_CamParameters(in.camParameters, out.cam_parameters);
}

void toStruct_CoopAwareness(const cam_msgs::CoopAwareness& in, CoopAwareness_t& out) {
  memset(&out, 0, sizeof(CoopAwareness_t));

  toStruct_GenerationDeltaTime(in.generation_delta_time, out.generationDeltaTime);
  toStruct_CamParameters(in.cam_parameters, out.camParameters);
}

}
