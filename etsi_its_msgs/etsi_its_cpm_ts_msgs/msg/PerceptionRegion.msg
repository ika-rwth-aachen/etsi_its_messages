# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
# Copyright (c) 2024 Instituto de Telecomunicações, Universidade de Aveiro
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

# --- Auto-generated by asn1ToRosMsg.py ----------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# PerceptionRegion ::= SEQUENCE {
#     measurementDeltaTime         DeltaTimeMilliSecondSigned,     
#     perceptionRegionConfidence   ConfidenceLevel,
#     perceptionRegionShape        Shape, 
#     shadowingApplies             BOOLEAN,
#     sensorIdList                 SequenceOfIdentifier1B OPTIONAL,
#     numberOfPerceivedObjects     CardinalNumber1B OPTIONAL,
#     perceivedObjectIds           PerceivedObjectIds OPTIONAL,
# 	...
# }
# ------------------------------------------------------------------------------
# *
# * This DF represents the actual perception capabilities available to the transmitting ITS-S, offering additional (often dynamic) details to the information provided in the sensor information container.
# *
# * It shall include the following components: 
#
# * @field measurementDeltaTime: difference between the time of estimation of the perception region and the reference time. Positive values indicates that the provided information refers to a point in time after the reference time.
# *
# * @field perceptionRegionConfidence: the perception confidence.
# *
# * @field perceptionRegionShape: specification of the shape of the perception region.
# *
# * @field shadowingApplies: indicates if the standard shadowing approach applies to the described perception region.
# *
# * @field sensorIdList: the optional list of identifiers of the sensors which are involved in perceiving the region.
# *
# * @field numberOfPerceivedObjects: the optional number of perceived objects contained in the perception region specified in the component perceptionRegionShape. 
# *
# * @field perceivedObjectIds: the optional list of identifiers of the objects specified in the Perceived Object Container that are contained in the perception region specified in the component perceptionRegionShape.
# *
#

DeltaTimeMilliSecondSigned measurement_delta_time

ConfidenceLevel perception_region_confidence

Shape perception_region_shape

bool shadowing_applies

SequenceOfIdentifier1B sensor_id_list
bool sensor_id_list_is_present

CardinalNumber1B number_of_perceived_objects
bool number_of_perceived_objects_is_present

PerceivedObjectIds perceived_object_ids
bool perceived_object_ids_is_present

