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
#   asn1/patched/uulm_mcm_etsi/TS103561_LUKAS_MCM.asn
#   asn1/patched/uulm_mcm_etsi/ETSI-ITS-CDD.asn
#   -t
#   mcm_uulm
#   -o
#   etsi_its_msgs/etsi_its_mcm_uulm_msgs/msg
# ------------------------------------------------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# ManeuverType ::= ENUMERATED {
# 	freeTravel           (0),
# 	followRoadUser       (1),
# 	crossIntersection    (2),
# 	yieldAtIntersection  (3),
# 	overtake             (4),
# 	pullOver             (5),
# 	yieldAtObstacle      (6),
# 	maneuverTypeUnknown  (15)
# }
# ------------------------------------------------------------------------------

uint8 value
uint8 FREE_TRAVEL = 0
uint8 FOLLOW_ROAD_USER = 1
uint8 CROSS_INTERSECTION = 2
uint8 YIELD_AT_INTERSECTION = 3
uint8 OVERTAKE = 4
uint8 PULL_OVER = 5
uint8 YIELD_AT_OBSTACLE = 6
uint8 MANEUVER_TYPE_UNKNOWN = 15

