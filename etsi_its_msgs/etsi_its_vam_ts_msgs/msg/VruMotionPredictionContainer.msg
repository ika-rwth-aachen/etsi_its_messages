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
#  /**
#  * @details VruMotionPredictionContainer
#  * The VRU Motion Prediction container of the VAM carries the past and future motion state information of the VRU.
#  * It includes the following components (setting indications are specified in clause 7.3.6 of TS 103 300-3). 
#  * At least one of the fields below shall be present if the container is present in the VAM: 
#  *
#  * @field pathHistory: represents the VRU's recent movement over some past time and/or distance. 
#  *        It consists of a list of path points. 
#  * @field pathPrediction: provides the set of predicted locations of the ITS-S, confidence values 
#  *        and the corresponding future time instants. 
#  * @field safeDistance: provides indication of safe distance between an ego-VRU and up to 8 other ITS-S 
#  *        or entity on the road to indicate whether the ego-VRU is at a safe distance (that is less likely to 
#  *        physically collide) from another ITS-S or entity on the road. 
#  * @field trajectoryInterceptionIndication: provides the indication for possible trajectory interception 
#  *        with up to 8 VRUs or other objects on the road.. 
#  * @field accelerationChangeIndication: provides an acceleration change indication of the VRU. 
#  *        When present this DF indicates an anticipated change in the VRU speed for period of actionDeltaTime.
#  * @field headingChangeIndication: provides additional data elements associated to heading change indicators 
#  *        such as a change of travel direction (left or right). 
#  *        The direction change action is performed for a period of actionDeltaTime. 
#  * @field stabilityChangeIndication: provides an estimation of the VRU stability. 
#  * 
#  * @category: GeoReference information, VRU information
#  * @revision: V2.2.1
#  */
#     VruMotionPredictionContainer ::= SEQUENCE {
#         pathHistory                      PathHistory OPTIONAL,
#         pathPrediction                   PathPredicted OPTIONAL,  
#         safeDistance                     SequenceOfSafeDistanceIndication OPTIONAL,
#         trajectoryInterceptionIndication SequenceOfTrajectoryInterceptionIndication OPTIONAL,
#         accelerationChangeIndication     AccelerationChangeIndication OPTIONAL,
#         headingChangeIndication          HeadingChangeIndication OPTIONAL,
#         stabilityChangeIndication        StabilityChangeIndication OPTIONAL,
#         ...
#     }
# ------------------------------------------------------------------------------

PathHistory path_history
bool path_history_is_present

PathPredicted path_prediction
bool path_prediction_is_present

SequenceOfSafeDistanceIndication safe_distance
bool safe_distance_is_present

SequenceOfTrajectoryInterceptionIndication trajectory_interception_indication
bool trajectory_interception_indication_is_present

AccelerationChangeIndication acceleration_change_indication
bool acceleration_change_indication_is_present

HeadingChangeIndication heading_change_indication
bool heading_change_indication_is_present

StabilityChangeIndication stability_change_indication
bool stability_change_indication_is_present

