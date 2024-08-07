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
# VruDeviceUsage ::= INTEGER {
#     unavailable      (0), 
#     other            (1), 
#     idle             (2), 
#     listeningToAudio (3), 
#     typing           (4), 
#     calling          (5), 
#     playingGames     (6), 
#     reading          (7), 
#     viewing          (8) 
# }(0..15)
# ------------------------------------------------------------------------------
# *
# * This DE represents the possible usage conditions of the VRU device.
#
# * - The value shall be set to:
# * - 0 `unavailable`      - to indicate that the usage conditions are unavailable,
# * - 1 `other`            - to indicate that the VRU device is in a state not defined below,
# * - 2 `idle`             - to indicate that the human is currently not interacting with the device,
# * - 3 `listeningToAudio` - to indicate that any audio source other than calling is in use,
# * - 4 `typing`           - to indicate that the human is texting or performaing any other manual input activity,
# * - 5 `calling`          - to indicate that the VRU device is currently receiving a call,
# * - 6 `playingGames`     - to indicate that the human is playing games,
# * - 7 `reading`          - to indicate that the human is reading on the VRU device,
# * - 8 `viewing`          - to indicate that the human is watching dynamic content, including following navigation prompts, viewing videos or other visual contents that are not static.
# * - value 9 to 15        - are reserved for future usage. 
# *
# * @category: VRU information
# * @revision: Created in V2.1.1, type changed from ENUMERATED to INTEGER in V2.2.1 and range changed from 0..255 to 0..15
# 

uint8 value
uint8 MIN = 0
uint8 MAX = 15
uint8 UNAVAILABLE = 0
uint8 OTHER = 1
uint8 IDLE = 2
uint8 LISTENING_TO_AUDIO = 3
uint8 TYPING = 4
uint8 CALLING = 5
uint8 PLAYING_GAMES = 6
uint8 READING = 7
uint8 VIEWING = 8
