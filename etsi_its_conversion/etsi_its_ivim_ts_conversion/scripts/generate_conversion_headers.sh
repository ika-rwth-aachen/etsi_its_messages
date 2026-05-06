#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"
OUT_DIR="${PKG_DIR}/include/etsi_its_ivim_ts_conversion"
RAW_DIR="${REPO_ROOT}/asn1/raw/is_ts103301"
PATCHED_DIR="${REPO_ROOT}/asn1/patched/is_ts103301"
AUDIT_SCRIPT="${SCRIPT_DIR}/audit_conversion_coverage.py"
ARCHIVE_VERSION="v2.1.1"
ARCHIVE_ZIP_URL="https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/archive/${ARCHIVE_VERSION}/is_ts103301-${ARCHIVE_VERSION}.zip"
ARCHIVE_TGZ_URL="https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/archive/${ARCHIVE_VERSION}/is_ts103301-${ARCHIVE_VERSION}.tar.gz"
CDD_RAW_URL="https://forge.etsi.org/rep/ITS/asn1/cdd_ts102894_2/-/raw/v1.3.1/ITS-Container.asn"

cd "${REPO_ROOT}"
PYTHON_CMD="${PYTHON_CMD:-python3}"
mkdir -p "${OUT_DIR}" "${REPO_ROOT}/asn1/raw" "${REPO_ROOT}/asn1/patched"

REQUIRED_RAW=(
  "IVIM-PDU-Descriptions.asn"
  "cdd/ITS-Container.asn"
  "iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
  "iso-patched/ISO14823-missing.asn"
  "build/asn1/TS17419_2014_CITSapplMgmtIDs.asn"
  "iso-patched/ISO14906(2018)EfcDsrcGenericv7-patched.asn"
  "iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
  "build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
  "build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn"
  "build/asn1/ISO19321IVIv2.asn"
)

need_bootstrap=0
for rel in "${REQUIRED_RAW[@]}"; do
  if [[ ! -f "${RAW_DIR}/${rel}" ]]; then
    need_bootstrap=1
    break
  fi
done

bootstrap_from_archive() {
  local tmpdir root
  tmpdir="$(mktemp -d)"
  trap 'rm -rf "${tmpdir}"' RETURN

  echo "Bootstrapping IVIM ASN.1 from ETSI is_ts103301 ${ARCHIVE_VERSION} ..."
  if command -v wget >/dev/null 2>&1; then
    if ! wget -q -O "${tmpdir}/is_ts103301.zip" "${ARCHIVE_ZIP_URL}"; then
      echo "zip download failed, trying tar.gz ..."
      wget -q -O "${tmpdir}/is_ts103301.tar.gz" "${ARCHIVE_TGZ_URL}"
      tar -xzf "${tmpdir}/is_ts103301.tar.gz" -C "${tmpdir}"
    else
      unzip -q "${tmpdir}/is_ts103301.zip" -d "${tmpdir}"
    fi
  elif command -v curl >/dev/null 2>&1; then
    if ! curl -fsSL "${ARCHIVE_ZIP_URL}" -o "${tmpdir}/is_ts103301.zip"; then
      echo "zip download failed, trying tar.gz ..."
      curl -fsSL "${ARCHIVE_TGZ_URL}" -o "${tmpdir}/is_ts103301.tar.gz"
      tar -xzf "${tmpdir}/is_ts103301.tar.gz" -C "${tmpdir}"
    else
      unzip -q "${tmpdir}/is_ts103301.zip" -d "${tmpdir}"
    fi
  else
    echo "Neither wget nor curl is available." >&2
    return 1
  fi

  root="$(find "${tmpdir}" -maxdepth 1 -mindepth 1 -type d | head -n 1)"
  if [[ -z "${root}" ]]; then
    echo "Failed to detect extracted ETSI archive root." >&2
    return 1
  fi

  rm -rf "${RAW_DIR}"
  mkdir -p "${RAW_DIR}"
  cp -a "${root}/." "${RAW_DIR}/"
  echo "ETSI IVIM ASN.1 extracted to ${RAW_DIR}"
}

download_cdd_if_missing() {
  if [[ -f "${RAW_DIR}/cdd/ITS-Container.asn" ]]; then
    return 0
  fi
  echo "Downloading CDD ITS-Container.asn ..."
  mkdir -p "${RAW_DIR}/cdd"
  if command -v wget >/dev/null 2>&1; then
    wget -q -O "${RAW_DIR}/cdd/ITS-Container.asn" "${CDD_RAW_URL}"
  elif command -v curl >/dev/null 2>&1; then
    curl -fsSL "${CDD_RAW_URL}" -o "${RAW_DIR}/cdd/ITS-Container.asn"
  else
    echo "Neither wget nor curl is available for downloading ITS-Container.asn." >&2
    return 1
  fi
}

prepare_external_assets() {
  echo "Preparing external ASN.1 assets ..."
  bash asn1/external/download.sh
  echo "Applying ASN.1 patches ..."
  bash asn1/patches/patch.sh
}

if [[ ${need_bootstrap} -eq 1 ]]; then
  bootstrap_from_archive
  download_cdd_if_missing
  prepare_external_assets
fi

# ensure the CDD file exists even when a previous partial bootstrap left the tree behind
if [[ ! -f "${RAW_DIR}/cdd/ITS-Container.asn" ]]; then
  download_cdd_if_missing
fi

# ensure patched ISO14816 file exists where the generator expects it
mkdir -p "${PATCHED_DIR}/build/asn1"
if [[ -f "${RAW_DIR}/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn" && ! -f "${PATCHED_DIR}/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn" ]]; then
  cp -f "${RAW_DIR}/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn" \
        "${PATCHED_DIR}/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn"
fi

FILES=(
  "asn1/raw/is_ts103301/IVIM-PDU-Descriptions.asn"
  "asn1/raw/is_ts103301/cdd/ITS-Container.asn"
  "asn1/raw/is_ts103301/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn"
  "asn1/raw/is_ts103301/iso-patched/ISO14823-missing.asn"
  "asn1/raw/is_ts103301/build/asn1/TS17419_2014_CITSapplMgmtIDs.asn"
  "asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcGenericv7-patched.asn"
  "asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
  "asn1/raw/is_ts103301/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
  "asn1/patched/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn"
  "asn1/raw/is_ts103301/build/asn1/ISO19321IVIv2.asn"
)

missing=0
for f in "${FILES[@]}"; do
  if [[ ! -f "${REPO_ROOT}/${f}" ]]; then
    echo "Missing ASN.1 file: ${f}" >&2
    missing=1
  fi
done
if [[ ${missing} -ne 0 ]]; then
  echo >&2
  echo "IVIM ASN.1 bootstrap is still incomplete under asn1/raw/is_ts103301." >&2
  exit 1
fi

find "${OUT_DIR}" -maxdepth 1 -type f -name 'convert*.h' -delete

# Normalize legacy ASN.1 encodings to UTF-8 for the Python generator
"${PYTHON_CMD}" - <<'PY'
from pathlib import Path
files = [
    Path("asn1/patched/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn"),
]
for p in files:
    if not p.exists():
        continue
    raw = p.read_bytes()
    try:
        raw.decode("utf-8")
    except UnicodeDecodeError:
        for enc in ("cp1252", "latin-1"):
            try:
                p.write_text(raw.decode(enc), encoding="utf-8")
                break
            except Exception:
                pass
PY

"${PYTHON_CMD}" utils/codegen/codegen-py/asn1ToConversionHeader.py \
  "${FILES[@]}" \
  -t ivim_ts \
  -o "${OUT_DIR}"

"${PYTHON_CMD}" "${SCRIPT_DIR}/postprocess_conversion_headers.py"

echo
"${PYTHON_CMD}" "${AUDIT_SCRIPT}" --repo-root "${REPO_ROOT}" --report "${PKG_DIR}/IVIM_CONVERSION_COVERAGE.txt"

echo
HEADER_COUNT=$(find "${OUT_DIR}" -maxdepth 1 -type f -name 'convert*.h' | wc -l)
echo "Generated header count: ${HEADER_COUNT}"
if [[ ${HEADER_COUNT} -eq 0 ]]; then
  echo "No IVIM conversion headers were generated." >&2
  exit 1
fi
