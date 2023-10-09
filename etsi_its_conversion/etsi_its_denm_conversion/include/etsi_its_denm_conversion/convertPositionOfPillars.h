#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_coding/PositionOfPillars.h>
#include <etsi_its_denm_coding/PosPillar.h>
#include <etsi_its_denm_conversion/convertPosPillar.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PosPillar.h>
#include <etsi_its_denm_msgs/PositionOfPillars.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/pos_pillar.hpp>
#include <etsi_its_denm_msgs/msg/position_of_pillars.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PositionOfPillars(const PositionOfPillars_t& in, denm_msgs::PositionOfPillars& out) {

  for (int i = 0; i < in.list.count; i++) {
    denm_msgs::PosPillar array;
    toRos_PosPillar(*(in.list.array[i]), array);
    out.array.push_back(array);
  }

}

void toStruct_PositionOfPillars(const denm_msgs::PositionOfPillars& in, PositionOfPillars_t& out) {

  memset(&out, 0, sizeof(PositionOfPillars_t));

  for (int i = 0; i < in.array.size(); i++) {
    PosPillar_t array;
    toStruct_PosPillar(in.array[i], array);
    PosPillar_t* array_ptr = new PosPillar_t(array);
    int status = asn_sequence_add(&out, array_ptr);
    if (status != 0) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }

}

}