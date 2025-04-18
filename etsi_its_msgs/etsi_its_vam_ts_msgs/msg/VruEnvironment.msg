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
#   asn1/raw/vam-ts103300_3/VAM-PDU-Descriptions.asn
#   asn1/patched/vam-ts103300_3/cdd/ETSI-ITS-CDD.asn
#   -t
#   vam_ts
#   -o
#   etsi_its_msgs/etsi_its_vam_ts_msgs/msg
# ------------------------------------------------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# /**
#  * This DE represents the possible VRU environment conditions.
#  *
#  * - The value shall be set to:
#  * - 0 `unavailable`            - to indicate that the information on the type of environment is unavailable,
#  * - 1 `intersectionCrossing`   - to indicate that the VRU is on an intersection or crossing,
#  * - 2 `zebraCrossing`          - to indicate that the VRU is on a  zebra crossing (crosswalk),
#  * - 3 `sidewalk`               - to indicate that the VRU is on a sidewalk,
#  * - 4 `onVehicleRoad`          - to indicate that the VRU is on a traffic lane,
#  * - 5 `protectedGeographicArea`- to indicate that the VRU is in a protected area.
#  * - value 5 to 255             - are reserved for future usage. Value 255 is set to "max" in order to bound the size of the encoded field.
#  *
#  * @category: VRU information
#  * @revision: Created in V2.1.1
#  */
# VruEnvironment ::= ENUMERATED {
#     unavailable             (0), 
#     intersectionCrossing    (1), 
#     zebraCrossing           (2), 
#     sidewalk                (3), 
#     onVehicleRoad           (4), 
#     protectedGeographicArea (5), 
#     max                     (255)
# }
# ------------------------------------------------------------------------------

uint8 value
uint8 UNAVAILABLE = 0
uint8 INTERSECTION_CROSSING = 1
uint8 ZEBRA_CROSSING = 2
uint8 SIDEWALK = 3
uint8 ON_VEHICLE_ROAD = 4
uint8 PROTECTED_GEOGRAPHIC_AREA = 5
uint8 MAX = 255

