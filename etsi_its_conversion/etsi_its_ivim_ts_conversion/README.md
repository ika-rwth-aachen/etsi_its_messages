# etsi_its_ivim_ts_conversion

IVIM conversion package scaffold plus generator/audit helpers.

## Important fix in this ZIP

The code generator now maps `-t ivim_ts` to the top-level ASN.1/ROS type `IVIM`.
Without this mapping, the generator tries to keep `convertIVIM_TS.h` instead of `convertIVIM.h`,
which leaves the package with zero usable IVIM conversion headers.

## Generate headers

Preferred project-wide invocation, matching the other ETSI message packages:

```bash
python3 utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/is_ts103301/IVIM-PDU-Descriptions.asn \
  asn1/raw/is_ts103301/cdd/ITS-Container.asn \
  asn1/raw/is_ts103301/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn \
  asn1/raw/is_ts103301/iso-patched/ISO14823-missing.asn \
  asn1/raw/is_ts103301/build/asn1/TS17419_2014_CITSapplMgmtIDs.asn \
  asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcGenericv7-patched.asn \
  asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn \
  asn1/raw/is_ts103301/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn \
  asn1/patched/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn \
  asn1/raw/is_ts103301/build/asn1/ISO19321IVIv2.asn \
  -t ivim_ts \
  -o etsi_its_conversion/etsi_its_ivim_ts_conversion/include/etsi_its_ivim_ts_conversion
```

Optional local convenience wrapper with bootstrap/audit:

```bash
bash etsi_its_conversion/etsi_its_ivim_ts_conversion/scripts/generate_conversion_headers.sh
```

## Audit coverage

```bash
python3 etsi_its_conversion/etsi_its_ivim_ts_conversion/scripts/audit_conversion_coverage.py   --repo-root "$PWD"   --report etsi_its_conversion/etsi_its_ivim_ts_conversion/IVIM_CONVERSION_COVERAGE.txt
```

## Verify regeneration and build

```bash
bash etsi_its_conversion/etsi_its_ivim_ts_conversion/scripts/verify_codegen_and_build.sh
```

For CI-style reproducibility checks, add `CHECK_GIT_DIFF=1` to fail when regeneration changes tracked files.
