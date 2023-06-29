#pragma once

#include <etsi_its_cam_coding/CauseCode.h>
#include <etsi_its_cam_conversion/convertCauseCodeType.h>
#include <etsi_its_cam_conversion/convertSubCauseCodeType.h>
#include <etsi_its_cam_msgs/CauseCode.h>


namespace etsi_its_cam_conversion {

void toRos_CauseCode(const CauseCode_t& in, etsi_its_cam_msgs::CauseCode& out) {

  toRos_CauseCodeType(in.causeCode, out.causeCode);
  toRos_SubCauseCodeType(in.subCauseCode, out.subCauseCode);
}

void toStruct_CauseCode(const etsi_its_cam_msgs::CauseCode& in, CauseCode_t& out) {
    
  memset(&out, 0, sizeof(CauseCode_t));

  toStruct_CauseCodeType(in.causeCode, out.causeCode);
  toStruct_SubCauseCodeType(in.subCauseCode, out.subCauseCode);
}

}