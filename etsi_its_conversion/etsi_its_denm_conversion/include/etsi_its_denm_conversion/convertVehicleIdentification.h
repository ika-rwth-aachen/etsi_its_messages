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
    toRos_WMInumber(*in.wMInumber, out.w_m_inumber);
    out.w_m_inumber_is_present = true;
  }

  if (in.vDS) {
    toRos_VDS(*in.vDS, out.v_ds);
    out.v_ds_is_present = true;
  }

}

void toStruct_VehicleIdentification(const denm_msgs::VehicleIdentification& in, VehicleIdentification_t& out) {

  memset(&out, 0, sizeof(VehicleIdentification_t));

  if (in.w_m_inumber_is_present) {
    WMInumber_t w_m_inumber;
    toStruct_WMInumber(in.w_m_inumber, w_m_inumber);
    out.wMInumber = new WMInumber_t(w_m_inumber);
  }

  if (in.v_ds_is_present) {
    VDS_t v_ds;
    toStruct_VDS(in.v_ds, v_ds);
    out.vDS = new VDS_t(v_ds);
  }

}

}