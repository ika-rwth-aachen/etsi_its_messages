//// INTEGER DangerousEndOfQueueSubCauseCode


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/DangerousEndOfQueueSubCauseCode.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/DangerousEndOfQueueSubCauseCode.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/dangerous_end_of_queue_sub_cause_code.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_DangerousEndOfQueueSubCauseCode(const DangerousEndOfQueueSubCauseCode_t& in, denm_msgs::DangerousEndOfQueueSubCauseCode& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_DangerousEndOfQueueSubCauseCode(const denm_msgs::DangerousEndOfQueueSubCauseCode& in, DangerousEndOfQueueSubCauseCode_t& out) {
  memset(&out, 0, sizeof(DangerousEndOfQueueSubCauseCode_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
