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
        description="Creates header files from ASN.1 definitions for conversion between C structs and ROS messages.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN.1 files")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN.1 type")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory")

    args = parser.parse_args()

    return args


def loadJinjaTemplates() -> Dict[str, jinja2.environment.Template]:
    """Loads the jinja templates for conversion headers.

    Templates available for types `CHOICE`, `CUSTOM`, `ENUMERATED`, `PRIMITIVE`, `SEQUENCE`, `SEQUENCE OF`.

    Returns:
        Dict[str, jinja2.environment.Template]: jinja templates
    """

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir), trim_blocks=False)
    jinja_templates = {}
    jinja_templates["CHOICE"] = jinja_env.get_template("convertChoiceType.h.jinja2")
    jinja_templates["CUSTOM"] = jinja_env.get_template("convertCustomType.h.jinja2")
    jinja_templates["ENUMERATED"] = jinja_env.get_template("convertEnumeratedType.h.jinja2")
    jinja_templates["PRIMITIVE"] = jinja_env.get_template("convertPrimitiveType.h.jinja2")
    jinja_templates["SEQUENCE"] = jinja_env.get_template("convertSequenceType.h.jinja2")
    jinja_templates["SEQUENCE OF"] = jinja_env.get_template("convertSequenceOfType.h.jinja2")

    return jinja_templates


def asn1TypeToConversionHeader(type_name: str, asn1_type: Dict, asn1_types: Dict[str, Dict], asn1_values: Dict[str, Dict], asn1_sets: Dict[str, Dict], asn1_classes: Dict[str, Dict], asn1_raw: Dict[str, str], etsi_type: str, jinja_templates: jinja2.environment.Template) -> str:
    """Converts parsed ASN.1 type information to a conversion header string.

    Args:
        type_name (str): type name
        asn1_type (Dict): type information
        asn1_types (Dict[str, Dict]): type information of all types by type
        asn1_values (Dict[str, Dict]): value information of all values by name
        asn1_sets (Dict[str, Dict]): set information of all sets by name
        asn1_classes (Dict[str, Dict]): class information of all classes by name
        asn1_raw (Dict[str, str]): raw string definition by type
        etsi_type (str): ETSI message type, e.g., `cam`
        jinja_templates (Dict[str, jinja2.environment.Template]): jinja template

    Returns:
        str: conversion header string
    """

    # select jinja template based on type
    if asn1_type["type"] in jinja_templates:
        jinja_template = jinja_templates[asn1_type["type"]]
    elif asn1_type["type"] in ASN1_PRIMITIVES_2_ROS:
        jinja_template = jinja_templates["PRIMITIVE"]
    elif asn1_type["type"] in asn1_types:
        jinja_template = jinja_templates["CUSTOM"]
    else:
        raise TypeError(f"No jinja template for type '{asn1_type['type']}'")

    # build jinja context based on asn1 type information
    jinja_context = asn1TypeToJinjaContext(type_name, asn1_type, asn1_types, asn1_values, asn1_sets, asn1_classes)
    if jinja_context is None:
        return None

    # add etsi type to context
    jinja_context["etsi_type"] = etsi_type

    # add raw asn1 definition as comment
    if type_name in asn1_raw:
        jinja_context["asn1_definition"] = asn1_raw[type_name].rstrip("\n")

    # add a dict entry for unique and sorted members (used for includes)
    seen = set()
    unique_sorted_members = []
    for member in jinja_context["members"]:
        if "asn1_type_name" in member and member["asn1_type_name"] not in seen:
            unique_sorted_members.append(member)
            seen.add(member["asn1_type_name"])
    jinja_context["unique_sorted_members"] = sorted(unique_sorted_members, key=lambda member: member["asn1_type_name"])

    # render jinja template with context
    header = jinja_template.render(jinja_context)

    return header


def exportConversionHeader(header: str, type_name: str, output_dir: str):
    """Exports a conversion header.

    Exports to `output_dir`/`type_name`.h.

    Args:
        ros_msg (str): conversion header string
        type_name (str): type name / file name
        output_dir (str): output directory
    """

    # create output directory
    os.makedirs(output_dir, exist_ok=True)

    # export ROS .msg using jinja template
    filename = os.path.join(output_dir, f"convert{validRosType(type_name)}.h")
    with open(filename, "w", encoding="utf-8") as file:
        file.write(header)

def findDependenciesOfConversionHeaders(parent_file_path: str, type: str, file_list: List[str] = []) -> List[str]:
    # duplicate list to avoid modifying the original list
    new_file_list = file_list.copy()

    # load contents of conversion file
    with open(parent_file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:

            if line.startswith(f"#include <etsi_its_{type}_conversion/convert"):
                msg_type = line.split("/")[1].split(".")[0]
                if msg_type not in new_file_list and os.path.isfile(f"{os.path.dirname(parent_file_path)}/{msg_type}.h"):
                    new_file_list.append(msg_type)
                    new_file_list = findDependenciesOfConversionHeaders(f"{os.path.dirname(parent_file_path)}/{msg_type}.h", type, new_file_list)

    # make sure there are no duplicates and sort alphabetically
    new_file_list = sorted(list(set(new_file_list)))

    return new_file_list

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

    # generate conversion headers
    jinja_templates = loadJinjaTemplates()
    for type_name, asn1_type in (pbar := tqdm(asn1_types.items(), desc="Generating conversion headers")):
        pbar.set_postfix_str(type_name)
        header = asn1TypeToConversionHeader(type_name, asn1_type, asn1_types, asn1_values, asn1_sets, asn1_classes, asn1_raw, args.type, jinja_templates)
        exportConversionHeader(header, type_name, args.output_dir)

    # remove all files that are not required for top-level message type
    print("Removing files not required for top-level message type ...")
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
    header_files = findDependenciesOfConversionHeaders(os.path.join(args.output_dir, f"convert{msg_type}.h"), args.type, [f"convert{msg_type}"])
    for f in glob.glob(os.path.join(args.output_dir, "*.h")):
        if os.path.splitext(os.path.basename(f))[0] not in header_files:
            os.remove(f)

    print(f"Generated {len(header_files)} conversion headers for {msg_type}")

if __name__ == "__main__":

    main()
