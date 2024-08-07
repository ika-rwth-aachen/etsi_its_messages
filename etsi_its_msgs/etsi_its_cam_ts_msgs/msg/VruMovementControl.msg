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
# VruMovementControl ::= INTEGER {
#     unavailable                (0), 
#     braking                    (1), 
#     hardBraking                (2), 
#     stopPedaling               (3), 
#     brakingAndStopPedaling     (4), 
#     hardBrakingAndStopPedaling (5), 
#     noReaction                 (6) 
# }(0..15)
# ------------------------------------------------------------------------------
# *
# *  This DE indicates the status of the possible human control over a VRU vehicle.
# *
# * The value shall be set to:
# * - 0 `unavailable`                 - to indicate that the information is unavailable,
# * - 1 `braking`                     - to indicate that the VRU is braking,
# * - 2 `hardBraking`                 - to indicate that the VRU is braking hard,
# * - 3 `stopPedaling`                - to indicate that the VRU stopped pedaling,
# * - 4 `brakingAndStopPedaling`      - to indicate that the VRU stopped pedaling an is braking,
# * - 5 `hardBrakingAndStopPedaling`  - to indicate that the VRU stopped pedaling an is braking hard,
# * - 6 `noReaction`                  - to indicate that the VRU is not changing its behavior.
# * - 7 to 15                         - are reserved for future usage. 
# *
# * @category: VRU information
# * @revision: Created in V2.1.1, type changed from ENUMERATED to INTEGER in V2.2.1 and range changed from 0..255 to 0..15
# 

uint8 value
uint8 MIN = 0
uint8 MAX = 15
uint8 UNAVAILABLE = 0
uint8 BRAKING = 1
uint8 HARD_BRAKING = 2
uint8 STOP_PEDALING = 3
uint8 BRAKING_AND_STOP_PEDALING = 4
uint8 HARD_BRAKING_AND_STOP_PEDALING = 5
uint8 NO_REACTION = 6
