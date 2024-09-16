#!/bin/bash

patches_dir="$(dirname "$(readlink -f "$0")")"
patched_dir="$(dirname "$patches_dir")/patched"
repos_dir="$(dirname "$patches_dir")/raw"
echo "Finding .patch files in '$patches_dir' to apply to repositories in '$repos_dir' with output in '$patched_dir' ..."

# loop over all .patch files in the patches directory
find "$patches_dir" -type f -name "*.patch" | while read -r patch_file; do

    # find the original repository path
    relative_path="$(dirname ${patch_file#$patches_dir/})"
    repo_path="$repos_dir/$relative_path"
    file_to_patch="$repo_path/$(basename ${patch_file%.patch})"

    # copy file to patch to temporary git repository
    tmp_repo_path="$(mktemp -d)"
    tmp_file_to_patch="$tmp_repo_path/$(basename ${patch_file%.patch})"
    cp -r "$repo_path"/* "$tmp_repo_path"
    git -C "$tmp_repo_path" init --quiet

    # apply the patch
    git -C "$tmp_repo_path" apply "$patch_file"

    # move patched file to the patched directory
    patched_path="$patched_dir/$relative_path"
    patched_file="$patched_path/$(basename ${patch_file%.patch})"
    mkdir -p "$patched_path"
    mv "$tmp_file_to_patch" "$patched_file"

    echo "  Patched '$file_to_patch' and saved to '$patched_file'"
done
