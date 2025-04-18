# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

# --- Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages ---
# python3
#   utils/codegen/codegen-py/asn1ToRosMsg.py
#   asn1/raw/cam_ts103900/CAM-PDU-Descriptions.asn
#   asn1/patched/cam_ts103900/cdd/ETSI-ITS-CDD.asn
#   -t
#   cam_ts
#   -o
#   etsi_its_msgs/etsi_its_cam_ts_msgs/msg
# ------------------------------------------------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# /**
#  * This DE indicates the steering wheel angle confidence value which represents the estimated absolute accuracy for a  steering wheel angle value with a confidence level of 95 %.
#  * 
#  * The value shall be set to:
#  * - `n` (`n > 0` and `n < 126`) if the confidence value is equal to or less than n x 1,5 degrees,
#  * - `126` if the confidence value is out of range, i.e. greater than 187,5 degrees,
#  * - `127` if the confidence value is not available.
#  * 
#  * @note: The fact that a steering wheel angle value is received with confidence value set to `unavailable(127)`
#  * can be caused by several reasons, such as:
#  * - the sensor cannot deliver the accuracy at the defined confidence level because it is a low-end sensor,
#  * - the sensor cannot calculate the accuracy due to lack of variables, or
#  * - there has been a vehicle bus (e.g. CAN bus) error.
#  * In all 3 cases above, the steering wheel angle value may be valid and used by the application.
#  * 
#  * If a steering wheel angle value is received and its confidence value is set to `outOfRange(126)`,
#  * it means that the steering wheel angle value is not valid and therefore cannot be trusted.
#  * Such value is not useful for the application.
#  * 
#  * @unit: 1,5 degree
#  * @category: Vehicle Information
#  * @revision: Description revised in V2.1.1
# */
# SteeringWheelAngleConfidence ::= INTEGER {
#     outOfRange                      (126), 
#     unavailable                     (127)
# } (1..127)
# ------------------------------------------------------------------------------

uint8 value
uint8 MIN = 1
uint8 MAX = 127
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127

