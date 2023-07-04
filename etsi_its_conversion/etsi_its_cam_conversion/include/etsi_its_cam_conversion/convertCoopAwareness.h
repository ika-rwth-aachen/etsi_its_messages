#pragma once

#include <etsi_its_cam_coding/CoopAwareness.h>
#include <etsi_its_cam_conversion/convertGenerationDeltaTime.h>
#include <etsi_its_cam_conversion/convertCamParameters.h>
#include <etsi_its_cam_msgs/CoopAwareness.h>


namespace etsi_its_cam_conversion {

void toRos_CoopAwareness(const CoopAwareness_t& in, etsi_its_cam_msgs::CoopAwareness& out) {

  toRos_GenerationDeltaTime(in.generationDeltaTime, out.generationDeltaTime);
  toRos_CamParameters(in.camParameters, out.camParameters);
}

void toStruct_CoopAwareness(const etsi_its_cam_msgs::CoopAwareness& in, CoopAwareness_t& out) {
    
  memset(&out, 0, sizeof(CoopAwareness_t));

  toStruct_GenerationDeltaTime(in.generationDeltaTime, out.generationDeltaTime);
  toStruct_CamParameters(in.camParameters, out.camParameters);
}

}