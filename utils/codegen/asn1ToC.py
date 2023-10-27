#!/usr/bin/env python3

import argparse
import glob
import os
import re
import shutil
import subprocess
import tempfile


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
    parser.add_argument("-di", "--docker-image", type=str, default="gitlab.ika.rwth-aachen.de:5050/fb-fi/definitions/etsi_its_messages/asn1c:latest", help="asn1c Docker image")

    args = parser.parse_args()

    return args


def adjustIncludes(parent_path: str):

    prefix = os.path.basename(parent_path)

    header_files = [os.path.join(parent_path, "include", prefix, f) for f in os.listdir(os.path.join(parent_path, "include", prefix))]
    source_files = [os.path.join(parent_path, "src", f) for f in os.listdir(os.path.join(parent_path, "src"))]
    header_files = [f for f in header_files if os.path.isfile(f)]
    source_files = [f for f in source_files if os.path.isfile(f)]
    headers = [os.path.basename(f) for f in header_files]

    for file in [*header_files, *source_files]:
        print(file)
        with open(file, "r") as f:
            contents = f.read()
            for header in headers:
                if re.search(r'^#include\s+"{}"'.format(header), contents, re.MULTILINE):
                    contents = re.sub(r'(^#include\s+")({}")'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
                if re.search(r'^#include\s+<{}>'.format(header), contents, re.MULTILINE):
                    contents = re.sub(r'(^#include\s+<)({}>)'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
        with open(file, "w") as f:
            f.write(contents)


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

            # copy input asn1 files to temporary directory
            for f in args.files:
                shutil.copy(f, temp_input_dir)

            # run asn1c docker container to generate header and source files
            subprocess.run(["docker", "run", "--rm", "-u", f"{os.getuid()}:{os.getgid()}", "-v", f"{temp_input_dir}:/input:ro", "-v", f"{temp_output_dir}:/output", args.docker_image], check=True)

            # move generated header and source files to output directories
            for f in glob.glob(os.path.join(temp_output_dir, "*.h")):
                shutil.move(f, os.path.join(output_include_dir, os.path.basename(f)))
            for f in glob.glob(os.path.join(temp_output_dir, "*.c")):
                shutil.move(f, os.path.join(output_source_dir, os.path.basename(f)))

    adjustIncludes(output_dir)


if __name__ == "__main__":

    main()
