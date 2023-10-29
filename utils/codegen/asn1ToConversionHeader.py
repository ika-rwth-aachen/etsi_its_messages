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
        description="Creates header files from ASN1 definitions for conversion between C structs and ROS messages.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN1 type")
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


def asn1TypeToConversionHeader(type_name: str, asn1_type: Dict, asn1_types: Dict[str, Dict], asn1_values: Dict[str, Dict], etsi_type: str, jinja_templates: jinja2.environment.Template) -> str:
    """Converts parsed ASN1 type information to a conversion header string.

    Args:
        type_name (str): type name
        asn1_type (Dict): type information
        asn1_types (Dict[str, Dict]): type information of all types by type
        asn1_values (Dict[str, Dict]): value information of all values by name
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
    jinja_context = asn1TypeToJinjaContext(type_name, asn1_type, asn1_types, asn1_values)
    if jinja_context is None:
        return None

    # add etsi type to context
    jinja_context["etsi_type"] = etsi_type

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
    print(filename)


def main():

    args = parseCli()

    asn1_docs, asn1_raw = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)
    asn1_values = extractAsn1ValuesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    jinja_templates = loadJinjaTemplates()

    for type_name, asn1_type in asn1_types.items():

        header = asn1TypeToConversionHeader(type_name, asn1_type, asn1_types, asn1_values, args.type, jinja_templates)

        exportConversionHeader(header, type_name, args.output_dir)


if __name__ == "__main__":

    main()
