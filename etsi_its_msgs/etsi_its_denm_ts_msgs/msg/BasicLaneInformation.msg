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
#  * This DF provides basic information about a single lane of a road segment.
#  * It includes the following components: 
#  * 
#  * @field laneNumber: the number associated to the lane that provides a transversal identification. 
#  * 
#  * @field direction: the direction of traffic flow allowed on the lane. 
#  * 
#  * @field laneWidth: the optional width of the lane.
#  *
#  * @field connectingLane: the number of the connecting lane in the next road section, i.e. the number of the lane which the vehicle will use when travelling from one section to the next,
#  * if it does not actively change lanes. If this component is absent, the lane name number remains the same in the next section.
#  *
#  * @field connectingRoadSection: the identifier of the next road section in direction of traffic, that is connecting to the current road section. 
#  * If this component is absent, the connecting road section is the one following the instance where this DF is placed in the @ref RoadConfigurationSectionList.
#  *
#  * @category: Road topology information
#  * @revision: Created in V2.2.1
# */
# BasicLaneInformation::= SEQUENCE{
#   laneNumber	         LanePosition,
#   direction		         Direction,
#   laneWidth		         LaneWidth OPTIONAL,
#   connectingLane         LanePosition OPTIONAL,
#   connectingRoadSection  RoadSectionId OPTIONAL,
#   ...
# }
# ------------------------------------------------------------------------------

LanePosition lane_number

Direction direction

LaneWidth lane_width
bool lane_width_is_present

LanePosition connecting_lane
bool connecting_lane_is_present

RoadSectionId connecting_road_section
bool connecting_road_section_is_present

