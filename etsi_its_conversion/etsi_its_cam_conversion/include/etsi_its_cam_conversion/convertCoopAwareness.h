#pragma once

#include <etsi_its_cam_coding/CoopAwareness.h>
#include <etsi_its_cam_msgs/CoopAwareness.h>
#include <etsi_its_cam_conversion/convertGenerationDeltaTime.h>
#include <etsi_its_cam_conversion/convertCamParameters.h>

namespace etsi_its_cam_conversion {
  
void convert_CoopAwarenesstoRos(const CoopAwareness_t& _CoopAwareness_in, etsi_its_cam_msgs::CoopAwareness& _CoopAwareness_out) {
  convert_GenerationDeltaTimetoRos(_CoopAwareness_in.generationDeltaTime, _CoopAwareness_out.generationDeltaTime);
  convert_CamParameterstoRos(_CoopAwareness_in.camParameters, _CoopAwareness_out.camParameters);
}

void convert_CoopAwarenesstoC(const etsi_its_cam_msgs::CoopAwareness& _CoopAwareness_in, CoopAwareness_t& _CoopAwareness_out) {
  memset(&_CoopAwareness_out, 0, sizeof(CoopAwareness_t));
  convert_GenerationDeltaTimetoC(_CoopAwareness_in.generationDeltaTime, _CoopAwareness_out.generationDeltaTime);
  convert_CamParameterstoC(_CoopAwareness_in.camParameters, _CoopAwareness_out.camParameters);
}

}