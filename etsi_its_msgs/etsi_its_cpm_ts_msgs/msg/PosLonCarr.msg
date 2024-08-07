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
# PosLonCarr ::= INTEGER {
#     outOfRange  (126),
#     unavailable (127)
# } (1..127)
# ------------------------------------------------------------------------------
# *
# * This DE represents the distance from the centre of vehicle front bumper to the right or left longitudinal carrier of vehicle.
# * The left/right carrier refers to the left/right as seen from a passenger sitting in the vehicle.
# *
# * The value shall be set to:
# * - `n` (`n > 0` and `n < 126`) for any aplicable value between 0,01 metre and 1,26 metres, 
# * - `126` for values equal to or higher than 1.25 metres,
# * - `127` if the information is unavailable.
# *
# * @unit 0,01 metre 
# * @category Vehicle information
# * @revision: description revised in V2.1.1 (the meaning of 126 has changed slightly) 
# 

uint8 value
uint8 MIN = 1
uint8 MAX = 127
uint8 OUT_OF_RANGE = 126
uint8 UNAVAILABLE = 127
