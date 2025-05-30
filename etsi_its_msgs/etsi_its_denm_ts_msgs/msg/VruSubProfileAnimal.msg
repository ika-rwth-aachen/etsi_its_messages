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
#  * This DE indicates the profile of an animal
#  * 
#  * The value shall be set to:
#  * - 0 `unavailable`     - to indicate that the information  is unavailable,
#  * - 1 `wild-animal`     - to indicate a animal living in the wildness, 
#  * - 2 `farm-animal`     - to indicate an animal beloning to a farm,
#  * - 3 `service-animal`  - to indicate an animal that supports a human being.
#  * - 4 to 15             - are reserved for future usage. 
#  *
#  * @category: VRU information
#  * @revision: Created in V2.1.1, type changed from ENUMERATED to INTEGER in V2.2.1
#  */
# VruSubProfileAnimal ::= INTEGER {
#     unavailable    (0), 
#     wild-animal    (1), 
#     farm-animal    (2), 
#     service-animal (3)
# }(0..15)
# ------------------------------------------------------------------------------

uint8 value
uint8 MIN = 0
uint8 MAX = 15
uint8 UNAVAILABLE = 0
uint8 WILD_ANIMAL = 1
uint8 FARM_ANIMAL = 2
uint8 SERVICE_ANIMAL = 3

