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
#  * This DF represents a set of options to describe a lane position and is the second level DF to represent a lane position. The top-level DFs are @ref GeneralizedLanePosition or @ref OccupiedLanesWithConfidence. 
#  * A lane position is a transversal position on the carriageway at a specific longitudinal position, in resolution of lanes of the carriageway.
#  *
#  * The following options are available:
#  *
#  * @field simplelanePosition: a single lane position without any additional context information.
#  *
#  * @field simpleLaneType: a lane type, to be used when the lane position is unknown but the type of lane is known. This can be used in scenarios where a certain confidence about the used lane type is given 
#  * but no or limited knowledge about the absolute lane number is available. For example, a cyclist on a cycle-lane or vehicles on a specific lane that is unique for the part of the road (e.g. a bus lane).
#  * 
#  * @field detailedlanePosition: a single lane position with additional lane details.
#  * 
#  * @field lanePositionWithLateralDetails: a single lane position with additional details and the lateral position within the lane.
#  *
#  * @field trafficIslandPosition: a position on a traffic island, i.e. between two lanes. 
#  *
#  * @category: Road Topology information
#  * @revision: Created in V2.2.1 from the DF GeneralizedLanePosition of V2.1.1. 
#  */
# LanePositionOptions ::= CHOICE {
#     simplelanePosition                   LanePosition,
#     simpleLaneType                       LaneType,
#     detailedlanePosition                 LanePositionAndType,
#     lanePositionWithLateralDetails       LanePositionWithLateralDetails,
#     trafficIslandPosition                TrafficIslandPosition,
#     ...
# }
# ------------------------------------------------------------------------------

uint8 choice

LanePosition simplelane_position
uint8 CHOICE_SIMPLELANE_POSITION = 0

LaneType simple_lane_type
uint8 CHOICE_SIMPLE_LANE_TYPE = 1

LanePositionAndType detailedlane_position
uint8 CHOICE_DETAILEDLANE_POSITION = 2

LanePositionWithLateralDetails lane_position_with_lateral_details
uint8 CHOICE_LANE_POSITION_WITH_LATERAL_DETAILS = 3

TrafficIslandPosition traffic_island_position
uint8 CHOICE_TRAFFIC_ISLAND_POSITION = 4

