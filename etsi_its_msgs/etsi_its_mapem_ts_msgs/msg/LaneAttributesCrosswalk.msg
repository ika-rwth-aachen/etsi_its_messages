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
#   asn1/raw/is_ts103301/MAPEM-PDU-Descriptions.asn
#   asn1/raw/is_ts103301/cdd/ITS-Container.asn
#   asn1/raw/is_ts103301/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn
#   asn1/raw/is_ts103301/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn
#   asn1/patched/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn
#   -t
#   mapem_ts
#   -o
#   etsi_its_msgs/etsi_its_mapem_ts_msgs/msg
# ------------------------------------------------------------------------------

# --- ASN.1 Definition ---------------------------------------------------------
# LaneAttributes-Crosswalk ::= BIT STRING { 
#   crosswalkRevocableLane  (0),
#   bicyleUseAllowed        (1),
#   isXwalkFlyOverLane      (2),
#   fixedCycleTime          (3),
#   biDirectionalCycleTimes (4),
#   hasPushToWalkButton     (5),
#   audioSupport            (6),
#   rfSignalRequestPresent  (7),
#   unsignalizedSegmentsPresent  (8)
# } (SIZE (16))
# ------------------------------------------------------------------------------

uint8[] value
uint8 bits_unused
uint8 SIZE_BITS = 16
uint8 BIT_INDEX_CROSSWALK_REVOCABLE_LANE = 0
uint8 BIT_INDEX_BICYLE_USE_ALLOWED = 1
uint8 BIT_INDEX_IS_XWALK_FLY_OVER_LANE = 2
uint8 BIT_INDEX_FIXED_CYCLE_TIME = 3
uint8 BIT_INDEX_BI_DIRECTIONAL_CYCLE_TIMES = 4
uint8 BIT_INDEX_HAS_PUSH_TO_WALK_BUTTON = 5
uint8 BIT_INDEX_AUDIO_SUPPORT = 6
uint8 BIT_INDEX_RF_SIGNAL_REQUEST_PRESENT = 7
uint8 BIT_INDEX_UNSIGNALIZED_SEGMENTS_PRESENT = 8

