#pragma once

#include <etsi_its_denm_coding/DangerousGoodsExtended.h>
#include <etsi_its_denm_conversion/convertDangerousGoodsBasic.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#include <etsi_its_denm_conversion/primitives/convertBOOLEAN.h>
#include <etsi_its_denm_conversion/primitives/convertBOOLEAN.h>
#include <etsi_its_denm_conversion/primitives/convertBOOLEAN.h>
#include <etsi_its_denm_conversion/primitives/convertIA5String.h>
#include <etsi_its_denm_conversion/convertPhoneNumber.h>
#include <etsi_its_denm_conversion/primitives/convertUTF8String.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/dangerous_goods_extended.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/DangerousGoodsExtended.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_DangerousGoodsExtended(const DangerousGoodsExtended_t& in, denm_msgs::DangerousGoodsExtended& out) {

  toRos_DangerousGoodsBasic(in.dangerousGoodsType, out.dangerous_goods_type);
  toRos_INTEGER(in.unNumber, out.un_number);
  toRos_BOOLEAN(in.elevatedTemperature, out.elevated_temperature);
  toRos_BOOLEAN(in.tunnelsRestricted, out.tunnels_restricted);
  toRos_BOOLEAN(in.limitedQuantity, out.limited_quantity);
  if (in.emergencyActionCode) {
    toRos_IA5String(*in.emergencyActionCode, out.emergency_action_code);
    out.emergency_action_code_is_present = true;
  }

  if (in.phoneNumber) {
    toRos_PhoneNumber(*in.phoneNumber, out.phone_number);
    out.phone_number_is_present = true;
  }

  if (in.companyName) {
    toRos_UTF8String(*in.companyName, out.company_name);
    out.company_name_is_present = true;
  }

}

void toStruct_DangerousGoodsExtended(const denm_msgs::DangerousGoodsExtended& in, DangerousGoodsExtended_t& out) {
    
  memset(&out, 0, sizeof(DangerousGoodsExtended_t));

  toStruct_DangerousGoodsBasic(in.dangerous_goods_type, out.dangerousGoodsType);
  toStruct_INTEGER(in.un_number, out.unNumber);
  toStruct_BOOLEAN(in.elevated_temperature, out.elevatedTemperature);
  toStruct_BOOLEAN(in.tunnels_restricted, out.tunnelsRestricted);
  toStruct_BOOLEAN(in.limited_quantity, out.limitedQuantity);
  if (in.emergency_action_code_is_present) {
    IA5String_t emergency_action_code;
    toStruct_IA5String(in.emergency_action_code, emergency_action_code);
    out.emergencyActionCode = new IA5String_t(emergency_action_code);
  }

  if (in.phone_number_is_present) {
    PhoneNumber_t phone_number;
    toStruct_PhoneNumber(in.phone_number, phone_number);
    out.phoneNumber = new PhoneNumber_t(phone_number);
  }

  if (in.company_name_is_present) {
    UTF8String_t company_name;
    toStruct_UTF8String(in.company_name, company_name);
    out.companyName = new UTF8String_t(company_name);
  }

}

}