/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/denm_ts103831/DENM-PDU-Descriptions.asn \
  asn1/patched/denm_ts103831/cdd/ETSI-ITS-CDD.asn \
  -t \
  denm_ts \
  -o \
  etsi_its_conversion/etsi_its_denm_ts_conversion/include/etsi_its_denm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DF shall contain a list of distances @ref PosPillar that refer to the perpendicular distance between centre of vehicle front bumper
 * and vehicle pillar A, between neighbour pillars until the last pillar of the vehicle.
 *
 * Vehicle pillars refer to the vertical or near vertical support of vehicle,
 * designated respectively as the A, B, C or D and other pillars moving in side profile view from the front to rear.
 * 
 * The first value of the DF refers to the perpendicular distance from the centre of vehicle front bumper to 
 * vehicle A pillar. The second value refers to the perpendicular distance from the centre position of A pillar
 * to the B pillar of vehicle and so on until the last pillar.
 *
 * @category: Vehicle information
 * @revision: V1.3.1
 *
PositionOfPillars ::= SEQUENCE (SIZE(1..3, ...)) OF PosPillar
----------------------------------------------------------------------------- */

#pragma once

#include <stdexcept>

#include <etsi_its_denm_ts_coding/asn_SEQUENCE_OF.h>
#include <etsi_its_denm_ts_coding/denm_ts_PositionOfPillars.h>
#include <etsi_its_denm_ts_coding/denm_ts_PosPillar.h>
#include <etsi_its_denm_ts_conversion/convertPosPillar.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/PosPillar.h>
#include <etsi_its_denm_ts_msgs/PositionOfPillars.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/pos_pillar.hpp>
#include <etsi_its_denm_ts_msgs/msg/position_of_pillars.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_PositionOfPillars(const denm_ts_PositionOfPillars_t& in, denm_ts_msgs::PositionOfPillars& out) {
  for (int i = 0; i < in.list.count; ++i) {
    denm_ts_msgs::PosPillar el;
    toRos_PosPillar(*(in.list.array[i]), el);
    out.array.push_back(el);
  }
}

void toStruct_PositionOfPillars(const denm_ts_msgs::PositionOfPillars& in, denm_ts_PositionOfPillars_t& out) {
  memset(&out, 0, sizeof(denm_ts_PositionOfPillars_t));
  for (int i = 0; i < in.array.size(); ++i) {
    denm_ts_PosPillar_t* el = (denm_ts_PosPillar_t*) calloc(1, sizeof(denm_ts_PosPillar_t));
    toStruct_PosPillar(in.array[i], *el);
    if (asn_sequence_add(&out, el)) throw std::invalid_argument("Failed to add to A_SEQUENCE_OF");
  }
}

}
