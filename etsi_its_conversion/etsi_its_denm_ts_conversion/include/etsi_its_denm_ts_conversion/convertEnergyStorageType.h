/** ============================================================================
MIT License

Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

/** Auto-generated by https://github.com/ika-rwth-aachen/etsi_its_messages -----
python3 \
  utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/denm_ts103831/DENM-PDU-Descriptions.asn \
  asn1/patched/denm_ts103831/cdd/ETSI-ITS-CDD.asn \
  -t \
  denm_ts \
  -o \
  etsi_its_conversion/etsi_its_denm_ts_conversion/include/etsi_its_denm_ts_conversion
----------------------------------------------------------------------------- */

/** ASN.1 Definition -----------------------------------------------------------
*
 * This DE indicated the type of energy being used and stored in vehicle.
 *
 * The corresponding bit shall be set to 1 under the following conditions:
 * - 0 - `hydrogenStorage`       - when hydrogen is being used and stored in vehicle,
 * - 1 - `electricEnergyStorage` - when electric energy is being used and stored in vehicle,
 * - 2 - `liquidPropaneGas`      - when liquid Propane Gas (LPG) is being used and stored in vehicle,   
 * - 3 - `compressedNaturalGas ` - when compressedNaturalGas (CNG) is being used and stored in vehicle,
 * - 4 - `diesel`                - when diesel is being used and stored in vehicle,
 * - 5 - `gasoline`              - when gasoline is being used and stored in vehicle,
 * - 6 - `ammonia`               - when ammonia is being used and stored in vehicle.
 *
 * - Otherwise, the corresponding bit shall be set to `0`.
 *
 * @category: Vehicle information
 * @revision: editorial revision in V2.1.1 
 *
EnergyStorageType ::= BIT STRING {
    hydrogenStorage       (0), 
    electricEnergyStorage (1), 
    liquidPropaneGas      (2), 
    compressedNaturalGas  (3), 
    diesel                (4), 
    gasoline              (5), 
    ammonia               (6)
}(SIZE(7))
----------------------------------------------------------------------------- */

#pragma once

#include <etsi_its_denm_ts_coding/denm_ts_EnergyStorageType.h>
#include <etsi_its_denm_ts_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_ts_msgs/EnergyStorageType.h>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs;
#else
#include <etsi_its_denm_ts_msgs/msg/energy_storage_type.hpp>
namespace denm_ts_msgs = etsi_its_denm_ts_msgs::msg;
#endif


namespace etsi_its_denm_ts_conversion {

void toRos_EnergyStorageType(const denm_ts_EnergyStorageType_t& in, denm_ts_msgs::EnergyStorageType& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_EnergyStorageType(const denm_ts_msgs::EnergyStorageType& in, denm_ts_EnergyStorageType_t& out) {
  memset(&out, 0, sizeof(denm_ts_EnergyStorageType_t));
  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
