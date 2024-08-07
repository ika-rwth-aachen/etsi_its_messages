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
# RoadSegmentReferenceId ::= SEQUENCE {
#     region    Identifier2B OPTIONAL,
#     id        Identifier2B
# }
# ------------------------------------------------------------------------------
# *
# * This DF represents a unique id for a road segment
# *
# * It shall include the following components: 
# *
# * @field region: the optional identifier of the entity that is responsible for the region in which the road segment is placed.
# * It is the duty of that entity to guarantee that the @ref Id is unique within the region.
# *
# * @field id: the identifier of the road segment.
# *
# * @note: when the component region is present, the RoadSegmentReferenceId is guaranteed to be globally unique.
# * @category: GeoReference information
# * @revision: created in V2.1.1
# 

Identifier2B region
bool region_is_present

Identifier2B id

