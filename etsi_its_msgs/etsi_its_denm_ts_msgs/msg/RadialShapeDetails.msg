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
#  * This DF describes radial shape details. The circular sector or cone is
#  * constructed by sweeping the provided range about the position specified outside of the  
#  * context of this DF. The range is swept between a horizontal start and a horizontal end angle in 
#  * the X-Y plane of a right-hand cartesian coordinate system specified outside of the context of 
#  * this DF, in positive angular direction w.r.t. the x-axis. A vertical opening angle in the X-Z 
#  * plane may optionally be provided in positive angular direction w.r.t. the x-axis.
#  * 
#  * It shall include the following components:
#  * 
#  * @field range: the radial range of the sensor from the reference point or sensor point offset. 
#  *
#  * @field horizontalOpeningAngleStart:  the start of the shape's horizontal opening angle.
#  *
#  * @field horizontalOpeningAngleEnd: the end of the shape's horizontal opening angle. 
#  *
#  * @field verticalOpeningAngleStart: optional start of the shape's vertical opening angle. 
#  *
#  * @field verticalOpeningAngleEnd: optional end of the shape's vertical opening angle. 
#  *
#  * @category: Georeference information
#  * @revision: created in V2.1.1, description revised and constraint added in V2.2.1
#  */
# RadialShapeDetails ::= SEQUENCE {
#     range                          StandardLength12b,
#     horizontalOpeningAngleStart    CartesianAngleValue,
#     horizontalOpeningAngleEnd      CartesianAngleValue,
#     verticalOpeningAngleStart      CartesianAngleValue OPTIONAL,
#     verticalOpeningAngleEnd        CartesianAngleValue OPTIONAL
# }
# ------------------------------------------------------------------------------

StandardLength12b range

CartesianAngleValue horizontal_opening_angle_start

CartesianAngleValue horizontal_opening_angle_end

CartesianAngleValue vertical_opening_angle_start
bool vertical_opening_angle_start_is_present

CartesianAngleValue vertical_opening_angle_end
bool vertical_opening_angle_end_is_present

