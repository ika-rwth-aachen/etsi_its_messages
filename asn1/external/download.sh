#!/bin/bash

patches_dir="$(dirname "$(readlink -f "$0")")"
repos_dir="$(dirname "$patches_dir")/raw"

# is_ts103301
cd "$repos_dir/is_ts103301"
sed "s/asn1c -D.*/exit 0/g" syntax_check.bash | bash
cd -
