#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"
VENV_DIR="${REPO_ROOT}/.venv_codegen"
PYTHON_BIN="${PYTHON_BIN:-${VENV_DIR}/bin/python}"
CHECK_GIT_DIFF="${CHECK_GIT_DIFF:-0}"
COLCON_BUILD_ARGS="${COLCON_BUILD_ARGS:-}"

cd "${REPO_ROOT}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  python3 -m venv "${VENV_DIR}"
fi

if ! "${PYTHON_BIN}" - <<'PY' >/dev/null 2>&1
import asn1tools
import jinja2
import numpy
import tqdm
PY
then
  "${PYTHON_BIN}" -m pip install --quiet --upgrade pip
  "${PYTHON_BIN}" -m pip install --quiet -r utils/codegen/codegen-py/requirements.txt
fi

"${PYTHON_BIN}" utils/codegen/codegen-py/asn1ToConversionHeader.py \
  asn1/raw/is_ts103301/IVIM-PDU-Descriptions.asn \
  asn1/raw/is_ts103301/cdd/ITS-Container.asn \
  asn1/raw/is_ts103301/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn \
  asn1/raw/is_ts103301/iso-patched/ISO14823-missing.asn \
  asn1/raw/is_ts103301/build/asn1/TS17419_2014_CITSapplMgmtIDs.asn \
  "asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcGenericv7-patched.asn" \
  "asn1/raw/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn" \
  asn1/raw/is_ts103301/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn \
  asn1/patched/is_ts103301/build/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn \
  asn1/raw/is_ts103301/build/asn1/ISO19321IVIv2.asn \
  -t ivim_ts \
  -o etsi_its_conversion/etsi_its_ivim_ts_conversion/include/etsi_its_ivim_ts_conversion

"${PYTHON_BIN}" etsi_its_conversion/etsi_its_ivim_ts_conversion/scripts/audit_conversion_coverage.py \
  --repo-root "${REPO_ROOT}" \
  --report etsi_its_conversion/etsi_its_ivim_ts_conversion/IVIM_CONVERSION_COVERAGE.txt

if [[ "${CHECK_GIT_DIFF}" == "1" ]]; then
  if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "CHECK_GIT_DIFF=1 requires a git worktree" >&2
    exit 1
  fi
  if [[ -n "$(git status --porcelain)" ]]; then
    echo "IVIM regeneration resulted in repository changes" >&2
    git diff -- etsi_its_conversion/etsi_its_ivim_ts_conversion utils/codegen/codegen-py .github/workflows
    exit 1
  fi
fi

if [[ -n "${COLCON_BUILD_ARGS}" ]]; then
  read -r -a colcon_args <<< "${COLCON_BUILD_ARGS}"
else
  colcon_args=(--packages-up-to etsi_its_conversion --symlink-install)
fi

colcon build "${colcon_args[@]}"
