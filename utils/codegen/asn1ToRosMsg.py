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
import os
from typing import Dict

import jinja2

from asn1CodeGenerationUtils import *


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


def asn1TypeToRosMsg(type_name: str, asn1_type: Dict, asn1_types: Dict[str, Dict], asn1_values: Dict[str, Dict], asn1_raw: Dict[str, str], jinja_template: jinja2.environment.Template) -> str:
    """Converts parsed ASN1 type information to a ROS message file string.

    Args:
        type_name (str): type name
        asn1_type (Dict): type information
        asn1_types (Dict[str, Dict]): type information of all types by type
        asn1_values (Dict[str, Dict]): value information of all values by name
        asn1_raw (Dict[str, str]): raw string definition by type
        jinja_template (jinja2.environment.Template): jinja template

    Returns:
        str: ROS message file string
    """

    # build jinja context based on asn1 type information
    jinja_context = asn1TypeToJinjaContext(type_name, asn1_type, asn1_types, asn1_values)
    if jinja_context is None:
        return None

    # add raw asn1 definition as comment to ROS .msg
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
    print(filename)


def main():

    args = parseCli()

    asn1_docs, asn1_raw = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)
    asn1_values = extractAsn1ValuesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    jinja_template = loadJinjaTemplate()

    for type_name, asn1_type in asn1_types.items():

        ros_msg = asn1TypeToRosMsg(type_name, asn1_type, asn1_types, asn1_values, asn1_raw, jinja_template)

        exportRosMsg(ros_msg, type_name, args.output_dir)


if __name__ == "__main__":

    main()
