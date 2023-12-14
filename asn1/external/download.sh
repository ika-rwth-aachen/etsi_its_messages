#!/bin/bash -e

# === MODIFIED FROM =================================================================
# https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/blob/v2.1.1/syntax_check.bash

# Copyright 2019 ETSI

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
# ===================================================================================

DIR="$(dirname "$(realpath "${BASH_SOURCE[-1]}")")"

if [ ! -f $DIR/ISO-TS-19091-addgrp-C-2018.asn ]; then
  wget -P $DIR https://standards.iso.org/iso/ts/19091/ed-2/en/ISO-TS-19091-addgrp-C-2018.asn
  sed -e 's/\bHeadingConfidence\b/HeadingConfidenceDSRC/g' \
      -e 's/\bSpeedConfidence\b/SpeedConfidenceDSRC/g' \
      -e 's/\bHeading\b/HeadingDSRC/g' \
  	  $DIR/ISO-TS-19091-addgrp-C-2018.asn > $DIR/ISO-TS-19091-addgrp-C-2018-patched.asn
fi
if [ ! -f "$DIR/ISO19321IVIv2.asn" ]; then
  wget -P $DIR 'https://standards.iso.org/iso/ts/19321/ed-2/en/ISO19321IVIv2.asn'
fi
if [ ! -f $DIR/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule_ForBallot.asn ]; then
  wget -P $DIR 'https://standards.iso.org/iso/24534/-3/ISO%2024534-3%20ASN.1%20repository/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule_ForBallot.asn'
fi
if [ ! -f $DIR/ISO14816_AVIAEINumberingAndDataStructures.asn ]; then
  wget -P $DIR https://standards.iso.org/iso/14816/ISO14816%20ASN.1%20repository/ISO14816_AVIAEINumberingAndDataStructures.asn
fi
if [ ! -f "$DIR/ISO14906(2018)EfcDsrcApplicationv6.asn" ]; then
  wget -P $DIR 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcApplicationv6.asn'
fi
if [ ! -f "$DIR/ISO14906(2018)EfcDsrcGenericv7.asn" ]; then
  wget -P $DIR 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcGenericv7.asn'
fi
if [ ! -f "$DIR/TS17419_2014_CITSapplMgmtIDs.asn" ]; then
  wget -P $DIR 'https://standards.iso.org/iso/ts/17419/TS%2017419%20ASN.1%20repository/TS17419_2014_CITSapplMgmtIDs.asn'
fi
if [ ! -f "$DIR/ISO14906(2018)EfcDsrcGenericv7.asn" ]; then
  wget -P $DIR 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcGenericv7.asn'
fi
