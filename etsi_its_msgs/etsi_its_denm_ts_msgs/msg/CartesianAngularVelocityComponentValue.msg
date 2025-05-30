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
#   asn1/raw/denm_ts103831/DENM-PDU-Descriptions.asn
#   asn1/patched/denm_ts103831/cdd/ETSI-ITS-CDD.asn
#   -t
#   denm_ts
#   -o
#   etsi_its_msgs/etsi_its_denm_ts_msgs/msg
# ------------------------------------------------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# /**
#  * This DE represents an angular velocity component described in a local Cartesian coordinate system, per default counted positive in
#  * a right-hand local coordinate system from the abscissa.
#  *
#  * The value shall be set to: 
#  * - `-255` if the velocity is equal to or less than -255 degrees/s,
#  * - `n` (`n > -255` and `n < 255`) if the velocity is equal to or less than n x 1 degree/s, and greater than (n-1) x 1 degree/s,
#  * - `255` if the velocity is greater than 254 degrees/s,
#  * - `256` if the information is unavailable.
#  *
#  * @unit: degree/s
#  * @category: Kinematic information
#  * @revision: Created in V2.1.1
# */
# CartesianAngularVelocityComponentValue ::= INTEGER {
#     negativeOutofRange (-255),
#     positiveOutOfRange (255),
#     unavailable	       (256)
# } (-255..256)
# ------------------------------------------------------------------------------

int16 value
int16 MIN = -255
int16 MAX = 256
int16 NEGATIVE_OUTOF_RANGE = -255
int16 POSITIVE_OUT_OF_RANGE = 255
int16 UNAVAILABLE = 256

