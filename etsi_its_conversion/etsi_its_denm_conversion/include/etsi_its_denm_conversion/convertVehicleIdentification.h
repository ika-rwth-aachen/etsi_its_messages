#pragma once

#include <etsi_its_denm_coding/VehicleIdentification.h>
#include <etsi_its_denm_conversion/convertWMInumber.h>
#include <etsi_its_denm_conversion/convertVDS.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleIdentification.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_identification.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleIdentification(const VehicleIdentification_t& in, denm_msgs::VehicleIdentification& out) {

  if (in.wMInumber) {
    toRos_WMInumber(*in.wMInumber, out.wm_inumber);
    out.wm_inumber_is_present = true;
  }

  if (in.vDS) {
    toRos_VDS(*in.vDS, out.vds);
    out.vds_is_present = true;
  }

}

void toStruct_VehicleIdentification(const denm_msgs::VehicleIdentification& in, VehicleIdentification_t& out) {

  memset(&out, 0, sizeof(VehicleIdentification_t));

  if (in.wm_inumber_is_present) {
    WMInumber_t wm_inumber;
    toStruct_WMInumber(in.wm_inumber, wm_inumber);
    out.wMInumber = new WMInumber_t(wm_inumber);
  }

  if (in.vds_is_present) {
    VDS_t vds;
    toStruct_VDS(in.vds, vds);
    out.vDS = new VDS_t(vds);
  }

}

}