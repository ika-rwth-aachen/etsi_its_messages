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
  asn1/raw/cam_ts103900/CAM-PDU-Descriptions.asn \
  asn1/patched/cam_ts103900/cdd/ETSI-ITS-CDD.asn \
  -t \
  cam_ts \
  -o \
  etsi_its_conversion/etsi_its_cam_ts_conversion/include/etsi_its_cam_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DE indicates the yaw rate confidence value which represents the estimated accuracy for a yaw rate value with a default confidence level of 95 %.
 * If required, the confidence level can be defined by the corresponding standards applying this DE.
 * 
 * The value shall be set to:
 * - `0` if the confidence value is equal to or less than 0,01 degree/second,
 * - `1` if the confidence value is equal to or less than 0,05 degrees/second or greater than 0,01 degree/second,
 * - `2` if the confidence value is equal to or less than 0,1 degree/second or greater than 0,05 degree/second,
 * - `3` if the confidence value is equal to or less than 1 degree/second or greater than 0,1 degree/second,
 * - `4` if the confidence value is equal to or less than 5 degrees/second or greater than 1 degrees/second,
 * - `5` if the confidence value is equal to or less than 10 degrees/second or greater than 5 degrees/second,
 * - `6` if the confidence value is equal to or less than 100 degrees/second or greater than 10 degrees/second,
 * - `7` if the confidence value is out of range, i.e. greater than 100 degrees/second,
 * - `8` if the confidence value is unavailable.
 * 
 * NOTE: The fact that a yaw rate value is received with confidence value set to `unavailable(8)` can be caused by
 * several reasons, such as:
 * - the sensor cannot deliver the accuracy at the defined confidence level because it is a low-end sensor,
 * - the sensor cannot calculate the accuracy due to lack of variables, or
 * - there has been a vehicle bus (e.g. CAN bus) error.
 * In all 3 cases above, the yaw rate value may be valid and used by the application.
 * 
 * If a yaw rate value is received and its confidence value is set to `outOfRange(7)`, it means that the 
 * yaw rate value is not valid and therefore cannot be trusted. Such value is not useful the application.
 * 
 * @category: Vehicle information
 * @revision: Description revised in V2.1.1
 *
YawRateConfidence ::= ENUMERATED {
    degSec-000-01 (0),
    degSec-000-05 (1),
    degSec-000-10 (2),
    degSec-001-00 (3), 
    degSec-005-00 (4),
    degSec-010-00 (5),
    degSec-100-00 (6),
    outOfRange    (7),
    unavailable   (8)
}
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_cam_ts_coding/cam_ts_YawRateConfidence.h>

#ifdef ROS1
#include <etsi_its_cam_ts_msgs/YawRateConfidence.h>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs;
#else
#include <etsi_its_cam_ts_msgs/msg/yaw_rate_confidence.hpp>
namespace cam_ts_msgs = etsi_its_cam_ts_msgs::msg;
#endif


namespace etsi_its_cam_ts_conversion {

void toRos_YawRateConfidence(const cam_ts_YawRateConfidence_t& in, cam_ts_msgs::YawRateConfidence& out) {
  out.value = in;
}

void toStruct_YawRateConfidence(const cam_ts_msgs::YawRateConfidence& in, cam_ts_YawRateConfidence_t& out) {
  memset(&out, 0, sizeof(cam_ts_YawRateConfidence_t));
  out = in.value;
}

}
