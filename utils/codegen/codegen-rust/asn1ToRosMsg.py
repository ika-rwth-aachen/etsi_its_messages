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
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN1 type")
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

def generate_rquired_msgs(parent_file_path: str, file_list: list = []) -> list:

    # load contents of msg file
    with open(parent_file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            # if line doesnt starts with # or empty line, get first word
            if not line.startswith("#") and line.strip() != "":
                msg_type = line.split()[0]
                # if ends with message type ends with [], remove []
                msg_type = msg_type[:-2] if msg_type.endswith("[]") else msg_type
                if msg_type not in file_list and os.path.isfile(f"{os.path.dirname(parent_file_path)}/{msg_type}.msg"):
                    file_list.append(msg_type)
                    generate_rquired_msgs(f"{os.path.dirname(parent_file_path)}/{msg_type}.msg", file_list)
    
    # make sure there are no duplicates and sort alphabetically
    file_list = list(set(file_list))
    
    
    return file_list

def generate_cmakelists(msg_files: list, file_path: str, type: str) -> None:
    with open(file_path, "w") as f:
        # Static content for CMake configuration
        f.write("cmake_minimum_required(VERSION 3.5)\n")
        f.write(f"project(etsi_its_{type}_msgs)\n\n")
        f.write("find_package(ros_environment REQUIRED QUIET)\n")
        f.write("set(ROS_VERSION $ENV{ROS_VERSION})\n\n")

        # ROS 2 (AMENT) section
        f.write("# === ROS 2 (AMENT) ============================================================\n")
        f.write("if(${ROS_VERSION} EQUAL 2)\n\n")
        f.write("  find_package(ament_cmake REQUIRED)\n")
        f.write("  find_package(rosidl_default_generators REQUIRED)\n\n")
        
        # Dynamically populate the msg files
        f.write("  set(msg_files\n")
        for msg_file in msg_files:
            f.write(f"    \"msg/{msg_file}.msg\"\n")
        f.write("  )\n\n")

        f.write("  rosidl_generate_interfaces(${PROJECT_NAME}\n")
        f.write("    ${msg_files}\n")
        f.write("  )\n\n")

        f.write("  ament_export_dependencies(rosidl_default_runtime)\n\n")
        f.write("  ament_package()\n\n")

        # ROS (CATKIN) section
        f.write("# === ROS (CATKIN) =============================================================\n")
        f.write("elseif(${ROS_VERSION} EQUAL 1)\n\n")
        f.write("  find_package(catkin REQUIRED COMPONENTS\n")
        f.write("    message_generation\n")
        f.write("    std_msgs\n")
        f.write("  )\n\n")

        f.write("  add_message_files(DIRECTORY msg)\n\n")
        f.write("  generate_messages(\n")
        f.write("    DEPENDENCIES std_msgs\n")
        f.write("  )\n\n")

        f.write("  catkin_package(\n")
        f.write("    CATKIN_DEPENDS\n")
        f.write("      message_runtime\n")
        f.write("      std_msgs\n")
        f.write("  )\n\n")

        f.write("endif()\n")

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

    msg_type = args.type.upper()
    
    # handle special cases
    if args.type == "cpm_ts":
        msg_type = "CollectivePerceptionMessage"
    elif args.type == "cam_ts":
        msg_type = "CAM"
    elif args.type == "vam_ts":
        msg_type = "VAM"

    msg_files = [msg_type]
    generate_rquired_msgs(os.path.join(args.output_dir, f"{msg_type}.msg"), msg_files)
    msg_files.sort()
    generate_cmakelists(msg_files, os.path.join(args.output_dir, "../CMakeLists.txt"), args.type)

if __name__ == "__main__":

    main()
