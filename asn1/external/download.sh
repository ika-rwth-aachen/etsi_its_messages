#!/bin/bash

patches_dir="$(dirname "$(readlink -f "$0")")"
repos_dir="$(dirname "$patches_dir")/raw"
is_ts103301_dir="$repos_dir/is_ts103301"
is_ts103301_archive_version="v2.1.1"
is_ts103301_archive_zip_url="https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/archive/${is_ts103301_archive_version}/is_ts103301-${is_ts103301_archive_version}.zip"
is_ts103301_archive_tgz_url="https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/archive/${is_ts103301_archive_version}/is_ts103301-${is_ts103301_archive_version}.tar.gz"
cdd_raw_url="https://forge.etsi.org/rep/ITS/asn1/cdd_ts102894_2/-/raw/v1.3.1/ITS-Container.asn"

download_file() {
    url="$1"
    output="$2"
    if command -v wget >/dev/null 2>&1; then
        wget -q -O "$output" "$url"
    elif command -v curl >/dev/null 2>&1; then
        curl -fsSL "$url" -o "$output"
    else
        echo "Neither wget nor curl is available." >&2
        return 1
    fi
}

bootstrap_is_ts103301_if_needed() {
    required_files=(
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
    for rel in "${required_files[@]}"; do
        if [ ! -f "$is_ts103301_dir/$rel" ]; then
            need_bootstrap=1
            break
        fi
    done

    if [ "$need_bootstrap" -eq 1 ]; then
        tmpdir="$(mktemp -d)"
        echo "Bootstrapping IVIM ASN.1 from ETSI is_ts103301 ${is_ts103301_archive_version} ..."
        if download_file "$is_ts103301_archive_zip_url" "$tmpdir/is_ts103301.zip"; then
            unzip -q "$tmpdir/is_ts103301.zip" -d "$tmpdir"
        else
            download_file "$is_ts103301_archive_tgz_url" "$tmpdir/is_ts103301.tar.gz"
            tar -xzf "$tmpdir/is_ts103301.tar.gz" -C "$tmpdir"
        fi
        archive_root="$(find "$tmpdir" -maxdepth 1 -mindepth 1 -type d | head -n 1)"
        if [ -z "$archive_root" ]; then
            echo "Failed to detect extracted ETSI archive root." >&2
            return 1
        fi
        mkdir -p "$is_ts103301_dir"
        cp -a "$archive_root/." "$is_ts103301_dir/"
        rm -rf "$tmpdir"
    fi

    if [ ! -f "$is_ts103301_dir/cdd/ITS-Container.asn" ]; then
        echo "Downloading CDD ITS-Container.asn ..."
        mkdir -p "$is_ts103301_dir/cdd"
        download_file "$cdd_raw_url" "$is_ts103301_dir/cdd/ITS-Container.asn"
    fi
}

# is_ts103301
bootstrap_is_ts103301_if_needed
cd "$is_ts103301_dir"
sed "s/asn1c -D.*/exit 0/g" syntax_check.bash | bash
cd -
