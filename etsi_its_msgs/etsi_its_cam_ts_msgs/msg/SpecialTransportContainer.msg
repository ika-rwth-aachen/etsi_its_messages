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
# SpecialTransportContainer ::= SEQUENCE {
# 	specialTransportType SpecialTransportType,
# 	lightBarSirenInUse LightBarSirenInUse
# }
# ------------------------------------------------------------------------------
# *
#* This type contains detaild information of the Special Transport Container.
#*
#* It shall include the following components:
#*
#* @field specialTransportType: which indicates whether the originating ITS-S is mounted on a special transport vehicle with heavy or oversized load
#* or both. It shall be present if the data is available in originating ITS-S.
#*
#* @field lightBarSirenInUse: indicates whether light-bar or a siren is in use by the vehicle originating the CAM.
#

SpecialTransportType special_transport_type

LightBarSirenInUse light_bar_siren_in_use

