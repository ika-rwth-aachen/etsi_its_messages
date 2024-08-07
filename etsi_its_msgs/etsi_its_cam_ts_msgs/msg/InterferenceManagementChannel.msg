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
# InterferenceManagementChannel ::= SEQUENCE {
#     centreFrequency    INTEGER (1 .. 99999),
#     channelWidth       INTEGER (0 .. 9999),
#     exponent           INTEGER (0 .. 15) 
# }
# ------------------------------------------------------------------------------
# *
# * This DF represents a frequency channel 
# *
# * It shall include the following components: 
# *
# * @field centreFrequency: the centre frequency of the channel in 10^(exp+2) Hz (where exp is exponent)
# * 
# * @field channelWidth: width of the channel in 10^exp Hz (where exp is exponent)
# *
# * @field exponent: exponent of the power of 10 used in the calculation of the components above.
# *
# * @category: Communication information
# * @revision: created in V2.1.1
#

uint32 centre_frequency
uint32 CENTRE_FREQUENCY_MIN = 1
uint32 CENTRE_FREQUENCY_MAX = 99999

uint16 channel_width
uint16 CHANNEL_WIDTH_MIN = 0
uint16 CHANNEL_WIDTH_MAX = 9999

uint8 exponent
uint8 EXPONENT_MIN = 0
uint8 EXPONENT_MAX = 15

