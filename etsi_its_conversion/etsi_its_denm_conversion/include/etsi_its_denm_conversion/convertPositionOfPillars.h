//// SEQUENCE-OF PositionOfPillars


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/PositionOfPillars.h>
#include <etsi_its_denm_conversion/convertPositionOfPillars.h>
#include <etsi_its_denm_conversion/convertPosPillar.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PositionOfPillars.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/position_of_pillars.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PositionOfPillars(const PositionOfPillars_t& in, denm_msgs::PositionOfPillars& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_msgs::PosPillar el;
    toRos_PosPillar(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_PositionOfPillars(const denm_msgs::PositionOfPillars& in, PositionOfPillars_t& out) {
  memset(&out, 0, sizeof(PositionOfPillars_t));

  for (int i = 0; i < in.array.size(); ++i) {
    PosPillar_t* el = (PosPillar_t*) calloc(1, sizeof(PosPillar_t));
    toStruct_PosPillar(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
