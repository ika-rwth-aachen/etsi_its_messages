#!/usr/bin/env python3

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
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
from typing import Dict, List


def parseCli():
    """Parses script's CLI arguments.

    Returns:
        argparse.Namespace: arguments
    """

    parser = argparse.ArgumentParser(
        description="Creates ROS .msg files from ASN1 definitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory")
    parser.add_argument("-td", "--temp-dir", type=str, default=None, help="temporary directory for mounting files to container; uses tempfile by default")
    parser.add_argument("-di", "--docker-image", type=str, default="ghcr.io/ika-rwth-aachen/etsi_its_messages:rgen", help="rgen Docker image")

    args = parser.parse_args()

    return args

def asn1Definitions(files: List[str]) -> Dict[str, str]:
    """Parses ASN1 files, extracts raw string definitions by type.

    Args:
        files (List[str]): filepaths

    Returns:
        Dict[str, str]]: raw string definition by type
    """

    asn1_raw = {}
    for file in files:
        with open(file) as f:
            lines = f.readlines()
        raw_def = None
        for line in lines:
            if "::=" in line:
                if line.rstrip().endswith("{"):
                    type = line.split("::=")[0].split("{")[0].strip().split()[0]
                    raw_def = ""
                elif len(line.split("::=")) == 2:
                    type = line.split("::=")[0].strip().split()[0]
                    if "}" in line or not ("{" in line or "}" in line):
                        raw_def = line
                        ros_type = type.replace("-", "")
                        asn1_raw[ros_type] = raw_def
                        raw_def = None
                    else:
                        raw_def = ""
            if raw_def is not None:
                raw_def += line
                if "}" in line and not "}," in line and not ("::=" in line and line.rstrip().endswith("{")):
                    ros_type = type.replace("-", "")
                    asn1_raw[ros_type] = raw_def
                    raw_def = None

    return asn1_raw


def main():

    args = parseCli()

    # create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # create temporary directories for running rgen in docker container
    with tempfile.TemporaryDirectory() as temp_input_dir:
        with tempfile.TemporaryDirectory() as temp_output_dir:

            if args.temp_dir is None:
                container_input_dir = temp_input_dir
                container_output_dir = temp_output_dir
            else:
                container_input_dir = os.path.join(args.temp_dir, "input")
                container_output_dir = os.path.join(args.temp_dir, "output")
                os.makedirs(container_input_dir, exist_ok=True)
                os.makedirs(container_output_dir, exist_ok=True)

            # copy input asn1 files to temporary directory
            for f in args.files:
                shutil.copy(f, container_input_dir)

            # run rgen docker container to generate .msg files
            subprocess.run(["docker", "run", "--rm", "-u", f"{os.getuid()}:{os.getgid()}", "-v", f"{container_input_dir}:/input:ro", "-v", f"{container_output_dir}:/output", args.docker_image, 'msgs', ""], check=True)

            # edit generated ROS .msg files to add auto-gen info, ASN.1 raw definitions (optional)
            asn1_raw = asn1Definitions(args.files)
            for f in glob.glob(os.path.join(container_output_dir, "*.msg")):
                with open(f, "r") as file:
                    msg = file.read()

                type = os.path.basename(f).split('.')[0]
                raw_def = asn1_raw[type]
                comments = "# --- Auto-generated by asn1ToRosMsg.py ----------------------------------------\n\n" +\
                           "# --- ASN.1 Definition ---------------------------------------------------------\n" +\
                           "\n".join(["# " + line for line in raw_def.split('\n')][:-1]) + '\n' +\
                           "# ------------------------------------------------------------------------------"
                msg = re.sub(r"^##\s([\w-]+)\s" + type + r"\b", comments, msg, flags=re.MULTILINE)

                with open(f, "w") as file:
                    file.write(msg)

            # move generated ROS .msg files to output directories
            for f in glob.glob(os.path.join(container_output_dir, "*.msg")):
                shutil.move(f, os.path.join(args.output_dir, os.path.basename(f)))


if __name__ == "__main__":

    main()
