#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023 Institute for Automotive Engineering (ika), RWTH Aachen University
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

import argparse
import glob
import os
import subprocess


def parseCli():

    parser = argparse.ArgumentParser(
        description="Applies patch files to raw ASN1 files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("-r", "--raw-dir", type=str, required=True, help="directory containing raw ASN1 files")
    parser.add_argument("-p", "--patch-dir", type=str, required=True, help="directory containing patch files with same name (+ .patch suffix) as raw ASN1 files")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory for patched ASN1 files")

    args = parser.parse_args()

    return args



def main():

    args = parseCli()

    # find all patch files
    patch_files = glob.glob(os.path.join(args.patch_dir, "**/*.patch"), recursive=True)

    # patch corresponding raw asn1 files
    for patch_file in patch_files:
        relative_patch_file_without_suffix = os.path.relpath(patch_file, args.patch_dir).replace(".patch", "")
        raw_file = os.path.join(args.raw_dir, relative_patch_file_without_suffix)
        if os.path.isfile(raw_file):
            output_file = os.path.join(args.output_dir, relative_patch_file_without_suffix)
            subprocess.run(["patch", raw_file, patch_file, "-o", output_file, "--quiet"], check=True)
            print(f"Patched '{raw_file}' with '{patch_file}' to '{output_file}'")


if __name__ == "__main__":

    main()
