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
from typing import Dict, List

import jinja2
from tqdm import tqdm

from asn1CodeGenerationUtils import *


def parseCli():
    """Parses script's CLI arguments.

    Returns:
        argparse.Namespace: arguments
    """

    parser = argparse.ArgumentParser(
        description="Creates ROS .msg files from ASN.1 definitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN.1 files")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN.1 type")

    args = parser.parse_args()

    return args


def loadJinjaTemplate() -> jinja2.environment.Template:
    """Loads the jinja template for ROS message files.

    Returns:
        jinja2.environment.Template: jinja template
    """

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir), trim_blocks=False)
    jinja_template = jinja_env.get_template("RosMessageType.msg.jinja2")

    return jinja_template


def asn1TypeToRosMsg(type_name: str, asn1_type: Dict, asn1_types: Dict[str, Dict], asn1_values: Dict[str, Dict], asn1_sets: Dict[str, Dict], asn1_classes: Dict[str, Dict], asn1_raw: Dict[str, str], jinja_template: jinja2.environment.Template) -> str:
    """Converts parsed ASN.1 type information to a ROS message file string.

    Args:
        type_name (str): type name
        asn1_type (Dict): type information
        asn1_types (Dict[str, Dict]): type information of all types by type
        asn1_values (Dict[str, Dict]): value information of all values by name
        asn1_sets (Dict[str, Dict]): set information of all sets by name
        asn1_classes (Dict[str, Dict]): class information of all classes by name
        asn1_raw (Dict[str, str]): raw string definition by type
        jinja_template (jinja2.environment.Template): jinja template

    Returns:
        str: ROS message file string
    """

    # build jinja context based on asn1 type information
    jinja_context = asn1TypeToJinjaContext(type_name, asn1_type, asn1_types, asn1_values, asn1_sets, asn1_classes)
    if jinja_context is None:
        return None

    # add raw asn1 definition as comment
    if type_name in asn1_raw:
        jinja_context["asn1_definition"] = asn1_raw[type_name].rstrip("\n")

    # render jinja template with context
    ros_msg = jinja_template.render(jinja_context)

    return ros_msg


def exportRosMsg(ros_msg: str, type_name: str, output_dir: str):
    """Exports a ROS message file.

    Exports to `output_dir`/`doc_name`/`type_name`.msg.

    Args:
        ros_msg (str): ROS message file string
        type_name (str): type name / file name
        doc_name (str): document name
        output_dir (str): output directory
    """

    # create output directory
    os.makedirs(output_dir, exist_ok=True)

    # export ROS .msg using jinja template
    filename = os.path.join(output_dir, f"{validRosType(type_name)}.msg")
    with open(filename, "w", encoding="utf-8") as file:
        file.write(ros_msg)

def findDependenciesOfRosMessageType(parent_file_path: str, file_list: List[str] = []) -> List[str]:
    # duplicate list to avoid modifying the original list
    new_file_list = file_list.copy()

    # load contents of msg file
    with open(parent_file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            # if line doesnt start with # or empty line, get first word
            if not line.startswith("#") and line.strip() != "":
                msg_type = line.split()[0]
                # if message type ends with [], remove []
                msg_type = msg_type[:-2] if msg_type.endswith("[]") else msg_type
                if msg_type not in new_file_list and os.path.isfile(f"{os.path.dirname(parent_file_path)}/{msg_type}.msg"):
                    new_file_list.append(msg_type)
                    new_file_list = findDependenciesOfRosMessageType(f"{os.path.dirname(parent_file_path)}/{msg_type}.msg", new_file_list)

    # make sure there are no duplicates and sort alphabetically
    new_file_list = sorted(list(set(new_file_list)))

    return new_file_list

def generateCMakeLists(msg_files: list, file_path: str, type: str) -> None:
    with open(file_path, "w") as f:
        msg_file_lines = "\n".join([f"    \"msg/{msg_file}.msg\"" for msg_file in msg_files])

    cmake_content = f"""\
cmake_minimum_required(VERSION 3.5)
project(etsi_its_{type}_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{{ROS_VERSION}})

# === ROS 2 (AMENT) ============================================================
if(${{ROS_VERSION}} EQUAL 2)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
{msg_file_lines}
  )

  rosidl_generate_interfaces(${{PROJECT_NAME}}
    ${{msg_files}}
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

# === ROS (CATKIN) =============================================================
elseif(${{ROS_VERSION}} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
  )

  add_message_files(DIRECTORY msg)

  generate_messages(
    DEPENDENCIES std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      message_runtime
      std_msgs
  )

endif()
"""

    # Write the entire content to the file in one operation
    with open(file_path, "w") as f:
        f.write(cmake_content)

def main():

    args = parseCli()

    # parse ASN.1 files
    print("Parsing ASN.1 files ...")
    asn1_docs, asn1_raw = parseAsn1Files(args.files)
    asn1_types = extractAsn1TypesFromDocs(asn1_docs)
    asn1_values = extractAsn1ValuesFromDocs(asn1_docs)
    asn1_sets = extractAsn1SetsFromDocs(asn1_docs)
    asn1_classes = extractAsn1ClassesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    # generate ROS .msg files
    jinja_template = loadJinjaTemplate()
    for type_name, asn1_type in (pbar := tqdm(asn1_types.items(), desc="Generating ROS .msg files")):
        pbar.set_postfix_str(type_name)
        ros_msg = asn1TypeToRosMsg(type_name, asn1_type, asn1_types, asn1_values, asn1_sets, asn1_classes, asn1_raw, jinja_template)
        exportRosMsg(ros_msg, type_name, args.output_dir)

    # generate CMakeLists.txt and remove all files that are not required for top-level message type
    msg_type = args.type.upper()
    if args.type == "cpm_ts":
        msg_type = "CollectivePerceptionMessage"
    elif args.type == "cam_ts":
        msg_type = "CAM"
    elif args.type == "mapem_ts":
        msg_type = "MAPEM"
    elif args.type == "spatem_ts":
        msg_type = "SPATEM"
    elif args.type == "vam_ts":
        msg_type = "VAM"
    msg_files = findDependenciesOfRosMessageType(os.path.join(args.output_dir, f"{msg_type}.msg"), [msg_type])
    print("Generating CMakeLists.txt ...")
    generateCMakeLists(msg_files, os.path.join(args.output_dir, "../CMakeLists.txt"), args.type)
    print("Removing files not required for top-level message type ...")
    for f in glob.glob(os.path.join(args.output_dir, "*.msg")):
        if os.path.splitext(os.path.basename(f))[0] not in msg_files:
            os.remove(f)

    print(f"Generated {len(msg_files)} ROS .msg files for {msg_type}")

if __name__ == "__main__":

    main()
