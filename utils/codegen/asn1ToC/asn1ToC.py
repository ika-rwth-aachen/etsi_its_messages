#!/usr/bin/env python3

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
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
import re
import shutil
import subprocess
import tempfile
from math import ceil
from multiprocessing import Pool

from tqdm import tqdm


def parseCli():
    """Parses script's CLI arguments.

    Returns:
        argparse.Namespace: arguments
    """

    parser = argparse.ArgumentParser(
        description="Creates header and source files from ASN1 definitions using asn1c.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output package directory")
    parser.add_argument("-td", "--temp-dir", type=str, default=None, help="temporary directory for mounting files to container; uses tempfile by default")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN1 type")
    parser.add_argument("-di", "--docker-image", type=str, default="ghcr.io/ika-rwth-aachen/etsi_its_messages:asn1c", help="asn1c Docker image")

    args = parser.parse_args()

    return args

def adjustIncludesInFile(args):
    """
    Adjusts the include statements in a file by adding a prefix to the included headers.

    Args:
        args (tuple): A tuple containing the file path, list of headers, and prefix.

    Returns:
        None
    """
    file, headers, prefix = args
    with open(file, "r") as f:
        contents = f.read()
        for header in headers:
            if re.search(r'^#include\s+"{}"'.format(header), contents, re.MULTILINE):
                contents = re.sub(r'(^#include\s+")({}")'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
            if re.search(r'^#include\s+<{}>'.format(header), contents, re.MULTILINE):
                contents = re.sub(r'(^#include\s+<)({}>)'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
    with open(file, "w") as f:
        f.write(contents)

def adjustIncludes(parent_path: str):

    prefix = os.path.basename(parent_path)
    header_files = [os.path.join(parent_path, "include", prefix, f) for f in os.listdir(os.path.join(parent_path, "include", prefix))]
    source_files = [os.path.join(parent_path, "src", f) for f in os.listdir(os.path.join(parent_path, "src"))]
    header_files = [f for f in header_files if os.path.isfile(f)]
    source_files = [f for f in source_files if os.path.isfile(f)]
    headers = [os.path.basename(f) for f in header_files]

    # process files in parallel
    files_to_process = [(file, headers, prefix) for file in [*header_files, *source_files]]
    with Pool(max(1, min(ceil(os.cpu_count() * 0.8), os.cpu_count() - 1))) as pool:
        dummy = list(tqdm(pool.imap(adjustIncludesInFile, files_to_process), total=len(files_to_process), desc="Adjusting includes"))

def main():

    args = parseCli()

    # create output directories
    output_dir = os.path.realpath(args.output_dir)
    output_include_dir = os.path.join(args.output_dir, "include", os.path.basename(output_dir))
    output_source_dir = os.path.join(args.output_dir, "src")
    os.makedirs(output_include_dir, exist_ok=True)
    os.makedirs(output_source_dir, exist_ok=True)

    # create temporary directories for running asn1c in docker container
    with tempfile.TemporaryDirectory() as temp_input_dir:
        with tempfile.TemporaryDirectory() as temp_output_dir:

            if args.temp_dir is None:
                container_input_dir = temp_input_dir
                container_output_dir = temp_output_dir
                asn1c_cmd_file = tempfile.NamedTemporaryFile(delete=False).name
            else:
                container_input_dir = os.path.join(args.temp_dir, "input")
                container_output_dir = os.path.join(args.temp_dir, "output")
                os.makedirs(container_input_dir, exist_ok=True)
                os.makedirs(container_output_dir, exist_ok=True)
                asn1c_cmd_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "asn1c.sh")

            # copy input asn1 files to temporary directory
            for f in args.files:
                shutil.copy(f, container_input_dir)

            # run asn1c docker container to generate header and source files
            with open(asn1c_cmd_file, "w") as f:
                f.write(f"asn1c $(find /input -name '*.asn' | sort) -fcompound-names -fprefix={args.type}_ -no-gen-BER -no-gen-XER -no-gen-OER -no-gen-example -gen-UPER -gen-JER")

            subprocess.run(["docker", "run", "--rm", "-u", f"{os.getuid()}:{os.getgid()}", "-v", f"{container_input_dir}:/input:ro", "-v", f"{container_output_dir}:/output", "-v", f"{asn1c_cmd_file}:/asn1c.sh", args.docker_image], check=True)
            os.remove(asn1c_cmd_file)

            # move generated header and source files to output directories
            for f in glob.glob(os.path.join(container_output_dir, "*.h")):
                shutil.move(f, os.path.join(output_include_dir, os.path.basename(f)))
            for f in glob.glob(os.path.join(container_output_dir, "*.c")):
                shutil.move(f, os.path.join(output_source_dir, os.path.basename(f)))

    adjustIncludes(output_dir)

    print(f"Applying patches ...")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    patch_file = os.path.join(script_dir, f"patches/{args.type}.patch")
    if os.path.exists(patch_file):
        subprocess.run(["git", "apply", patch_file], check=True)

    print(f"Generated C/C++ library for {args.type}")

if __name__ == "__main__":

    main()
