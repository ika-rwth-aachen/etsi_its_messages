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
from pathlib import Path
from typing import Dict, List

import jinja2
from tqdm import tqdm

from asn1CodeGenerationUtils import *


IVIM_ROS_TYPE_ALIASES = {
    "TrailerDetails": "TrailerCharacteristicsFixValuesList",
    "TrailerAxles": "TrailerCharacteristicsRangesList",
}

IVIM_CONVERSION_TYPE_ALIASES = {
    "DDDIO": "DDD_IO",
    "DDDIOLIST": "DDD_IO_LIST",
    "InternationalSignapplicablePeriod": "InternationalSign_applicablePeriod",
    "InternationalSigndistanceBetweenVehicles": "InternationalSign_distanceBetweenVehicles",
    "InternationalSignexemptedApplicablePeriod": "InternationalSign_exemptedApplicablePeriod",
    "InternationalSigndestinationInformation": "InternationalSign_destinationInformation",
    "DriverCharacteristics": "IVI_DriverCharacteristics",
    "TrailerCharacteristics": "IVI_TrailerCharacteristics",
    "StationType": "ITS_Container_StationType",
    "Temperature": "IVI_Temperature",
    "Temperature2": "IVI_Temperature",
}

IVIM_CONVERSION_HEADER_ALIASES = {
    "BIT STRING": "BIT_STRING",
    "OCTET STRING": "OCTET_STRING",
    "DDDIO": "DDD-IO",
    "DDDIOLIST": "DDD-IO-LIST",
    "InternationalSignapplicablePeriod": "InternationalSign-applicablePeriod",
    "InternationalSigndistanceBetweenVehicles": "InternationalSign-distanceBetweenVehicles",
    "InternationalSignexemptedApplicablePeriod": "InternationalSign-exemptedApplicablePeriod",
    "InternationalSigndestinationInformation": "InternationalSign-destinationInformation",
    "DriverCharacteristics": "IVI_DriverCharacteristics",
    "TrailerCharacteristics": "IVI_TrailerCharacteristics",
    "StationType": "ITS-Container_StationType",
    "Temperature": "IVI_Temperature",
    "Temperature2": "IVI_Temperature",
}


def applyRosTypeAliases(value, etsi_type: str):
    if etsi_type != "ivim_ts":
        return value
    if isinstance(value, dict):
        aliased = {k: applyRosTypeAliases(v, etsi_type) for k, v in value.items()}
        if "ros_msg_type" in aliased and "ros2_msg_type_file_name" in aliased:
            aliased["ros2_msg_type_file_name"] = validRosTypeHeader(aliased["ros_msg_type"].replace("[]", ""))
        return aliased
    if isinstance(value, list):
        return [applyRosTypeAliases(v, etsi_type) for v in value]
    if isinstance(value, str):
        is_array = value.endswith("[]")
        base = value[:-2] if is_array else value
        mapped = IVIM_ROS_TYPE_ALIASES.get(base, base)
        return f"{mapped}[]" if is_array else mapped
    return value


def applyEtsiTypePlaceholder(value, etsi_type: str):
    if isinstance(value, dict):
        return {k: applyEtsiTypePlaceholder(v, etsi_type) for k, v in value.items()}
    if isinstance(value, list):
        return [applyEtsiTypePlaceholder(v, etsi_type) for v in value]
    if isinstance(value, str):
        return value.replace("ETSI_TYPE_PLACEHOLDER", etsi_type)
    return value



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

    Templates available for types `CHOICE`, `CUSTOM`, `ENUMERATED`, `NULL`, `PRIMITIVE`, `SEQUENCE`, `SEQUENCE OF`.

    Returns:
        Dict[str, jinja2.environment.Template]: jinja templates
    """

    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir), trim_blocks=False)
    jinja_templates = {}
    jinja_templates["CHOICE"] = jinja_env.get_template("convertChoiceType.h.jinja2")
    jinja_templates["CUSTOM"] = jinja_env.get_template("convertCustomType.h.jinja2")
    jinja_templates["ENUMERATED"] = jinja_env.get_template("convertEnumeratedType.h.jinja2")
    jinja_templates["NULL"] = jinja_env.get_template("convertNullType.h.jinja2")
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
    jinja_context = applyEtsiTypePlaceholder(jinja_context, etsi_type)

    # align a few IVIM ROS type names with the existing manually curated .msg files
    jinja_context = applyRosTypeAliases(jinja_context, etsi_type)

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

    if header is None:
        return

    # create output directory
    os.makedirs(output_dir, exist_ok=True)

    # export ROS .msg using jinja template
    ros_type_name = validRosType(type_name)
    ros_type_name = applyRosTypeAliases(ros_type_name, "ivim_ts") if output_dir.endswith("etsi_its_ivim_ts_conversion") else ros_type_name
    filename = os.path.join(output_dir, f"convert{ros_type_name}.h")
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

    return new_file_list

def additionalMessageTypes(path: str, msg_type: str) -> List[str]:
    additional = []

    for f in glob.glob(os.path.join(path, "*.h")):
        msg_filename = os.path.splitext(os.path.basename(f))[0]
        if msg_type == "DENM" and msg_filename.endswith("SubCauseCode"):
            additional.append(msg_filename)

    return additional

def sortHeaderFiles(files: List[str]) -> List[str]:
    # make sure there are no duplicates and sort alphabetically
    return sorted(list(set(files)))


def normalizeInputFiles(args):
    if args.type != "ivim_ts":
        return

    for file_name in args.files:
        path = Path(file_name)
        if not path.exists():
            continue
        raw = path.read_bytes()
        try:
            raw.decode("utf-8")
        except UnicodeDecodeError:
            for encoding in ("cp1252", "latin-1"):
                try:
                    path.write_text(raw.decode(encoding), encoding="utf-8")
                    break
                except UnicodeDecodeError:
                    continue


def rewriteIvimConversionHeader(name: str, text: str) -> str:
    for src, dst in IVIM_CONVERSION_HEADER_ALIASES.items():
        text = text.replace(f"ivim_ts_{src}.h", f"ivim_ts_{dst}.h")
    for src, dst in IVIM_CONVERSION_TYPE_ALIASES.items():
        text = text.replace(f"ivim_ts_{src}_t", f"ivim_ts_{dst}_t")
        text = text.replace(f"sizeof(ivim_ts_{src}_t)", f"sizeof(ivim_ts_{dst}_t)")

    text = text.replace("ivim_ts_IVI-TrailerCharacteristics.h", "ivim_ts_IVI_TrailerCharacteristics.h")
    text = text.replace("ivim_ts_IVI-DriverCharacteristics.h", "ivim_ts_IVI_DriverCharacteristics.h")
    text = text.replace("ivim_ts_IVI-Temperature.h", "ivim_ts_IVI_Temperature.h")
    text = text.replace("ivim_ts_InternationalSign_destinationInformation.h", "ivim_ts_InternationalSign-destinationInformation.h")
    text = text.replace("ivim_ts_InternationalSign_distanceBetweenVehicles.h", "ivim_ts_InternationalSign-distanceBetweenVehicles.h")
    text = text.replace("ivim_ts_InternationalSign_exemptedApplicablePeriod.h", "ivim_ts_InternationalSign-exemptedApplicablePeriod.h")

    text = text.replace("toRos_BIT STRING", "toRos_BIT_STRING")
    text = text.replace("toStruct_BIT STRING", "toStruct_BIT_STRING")
    text = text.replace("toRos_OCTET STRING", "toRos_OCTET_STRING")
    text = text.replace("toStruct_OCTET STRING", "toStruct_OCTET_STRING")
    text = text.replace("convertBIT STRING.h", "convertBIT_STRING.h")
    text = text.replace("convertOCTET STRING.h", "convertOCTET_STRING.h")
    text = re.sub(r'#include <etsi_its_ivim_ts_conversion/convertuint\d+\.h>\n', "", text)
    text = re.sub(r'\btoRos_uint\d+\(([^,]+),\s*([^)]+)\);', r'\2 = \1;', text)
    text = re.sub(r'\btoStruct_uint\d+\(([^,]+),\s*([^)]+)\);', r'\2 = \1;', text)

    if name == "convertText.h":
        text = re.sub(
            r'  for \(int i = 0; i < in\.language\.list\.count; \+\+i\) \{\n'
            r'    ivim_ts_msgs::uint8 el;\n'
            r'    el = \*\(in\.language\.list\.array\[i\]\);\n'
            r'    out\.language\.push_back\(el\);\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toRos_BIT_STRING(in.language, out.language);\n"
            "  out.bits_unused = in.language.bits_unused;\n",
            text,
        )
        text = re.sub(
            r'    for \(int i = 0; i < in\.language\.size\(\); \+\+i\) \{\n'
            r'      ivim_ts_uint8_t\* el = \(ivim_ts_uint8_t\*\) calloc\(1, sizeof\(ivim_ts_uint8_t\)\);\n'
            r'      \*el = in\.language\[i\];\n'
            r'      if \(asn_sequence_add\(&out\.language, el\)\) throw std::invalid_argument\("Failed to add to A_SEQUENCE_OF"\);\n'
            r'    \}\n',
            "  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.language, out.language);\n"
            "  out.language.bits_unused = in.bits_unused;\n",
            text,
        )

    if name == "convertTcPart.h":
        text = re.sub(
            r'  for \(int i = 0; i < in\.data\.list\.count; \+\+i\) \{\n'
            r'    ivim_ts_msgs::uint8 el;\n'
            r'    el = \*\(in\.data\.list\.array\[i\]\);\n'
            r'    out\.data\.push_back\(el\);\n'
            r'  \}\n'
            r'  toRos_IviType\(in\.iviType, out\.ivi_type\);\n'
            r'  if \(in\.laneStatus\) \{\n'
            r'    toRos_LaneStatus\(\*in\.laneStatus, out\.lane_status\);\n'
            r'    out\.lane_status_is_present = true;\n'
            r'  \}\n'
            r'  if \(in\.vehicleCharacteristics\) \{\n'
            r'    toRos_VehicleCharacteristicsList\(\*in\.vehicleCharacteristics, out\.vehicle_characteristics\);\n'
            r'    out\.vehicle_characteristics_is_present = true;\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toRos_OCTET_STRING(in.data, out.data);\n"
            "  if (in.ext1) {\n"
            "    toRos_IviType(in.ext1->iviType, out.ivi_type);\n"
            "    if (in.ext1->laneStatus) {\n"
            "      toRos_LaneStatus(*in.ext1->laneStatus, out.lane_status);\n"
            "      out.lane_status_is_present = true;\n"
            "    }\n"
            "    if (in.ext1->vehicleCharacteristics) {\n"
            "      toRos_VehicleCharacteristicsList(*in.ext1->vehicleCharacteristics, out.vehicle_characteristics);\n"
            "      out.vehicle_characteristics_is_present = true;\n"
            "    }\n"
            "  }\n",
            text,
        )
        text = re.sub(
            r'    for \(int i = 0; i < in\.data\.size\(\); \+\+i\) \{\n'
            r'      ivim_ts_uint8_t\* el = \(ivim_ts_uint8_t\*\) calloc\(1, sizeof\(ivim_ts_uint8_t\)\);\n'
            r'      \*el = in\.data\[i\];\n'
            r'      if \(asn_sequence_add\(&out\.data, el\)\) throw std::invalid_argument\("Failed to add to A_SEQUENCE_OF"\);\n'
            r'    \}\n'
            r'  toStruct_IviType\(in\.ivi_type, out\.iviType\);\n'
            r'  if \(in\.lane_status_is_present\) \{\n'
            r'    out\.laneStatus = \(ivim_ts_LaneStatus_t\*\) calloc\(1, sizeof\(ivim_ts_LaneStatus_t\)\);\n'
            r'    toStruct_LaneStatus\(in\.lane_status, \*out\.laneStatus\);\n'
            r'  \}\n'
            r'  if \(in\.vehicle_characteristics_is_present\) \{\n'
            r'    out\.vehicleCharacteristics = \(ivim_ts_VehicleCharacteristicsList_t\*\) calloc\(1, sizeof\(ivim_ts_VehicleCharacteristicsList_t\)\);\n'
            r'    toStruct_VehicleCharacteristicsList\(in\.vehicle_characteristics, \*out\.vehicleCharacteristics\);\n'
            r'  \}\n',
            "  etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.data, out.data);\n"
            "  if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n"
            "  toStruct_IviType(in.ivi_type, out.ext1->iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.ext1->laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.ext1->laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.ext1->vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.ext1->vehicleCharacteristics);\n"
            "  }\n",
            text,
        )
        text = text.replace(
            "  toStruct_IviType(in.ivi_type, out.iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.vehicleCharacteristics);\n"
            "  }\n",
            "  if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n"
            "  toStruct_IviType(in.ivi_type, out.ext1->iviType);\n"
            "  if (in.lane_status_is_present) {\n"
            "    out.ext1->laneStatus = (ivim_ts_LaneStatus_t*) calloc(1, sizeof(ivim_ts_LaneStatus_t));\n"
            "    toStruct_LaneStatus(in.lane_status, *out.ext1->laneStatus);\n"
            "  }\n"
            "  if (in.vehicle_characteristics_is_present) {\n"
            "    out.ext1->vehicleCharacteristics = (ivim_ts_VehicleCharacteristicsList_t*) calloc(1, sizeof(ivim_ts_VehicleCharacteristicsList_t));\n"
            "    toStruct_VehicleCharacteristicsList(in.vehicle_characteristics, *out.ext1->vehicleCharacteristics);\n"
            "  }\n",
        )

    if name == "convertLaneInformation.h":
        for c_name, _, conv in [
            ("detectionZoneIds", "detection_zone_ids", "ZoneIds"),
            ("relevanceZoneIds", "relevance_zone_ids", "ZoneIds"),
            ("laneCharacteristics", "lane_characteristics", "LaneCharacteristics"),
            ("laneSurfaceStaticCharacteristics", "lane_surface_static_characteristics", "RoadSurfaceStaticCharacteristics"),
            ("laneSurfaceDynamicCharacteristics", "lane_surface_dynamic_characteristics", "RoadSurfaceDynamicCharacteristics"),
        ]:
            text = text.replace(f"if (in.{c_name}) {{", f"if (in.ext1 && in.ext1->{c_name}) {{")
            text = text.replace(f"*in.{c_name}", f"*in.ext1->{c_name}")
            text = text.replace(f"out.{c_name} = (ivim_ts_{conv}_t*) calloc(1, sizeof(ivim_ts_{conv}_t));", f"if (!out.ext1) out.ext1 = static_cast<decltype(out.ext1)>(calloc(1, sizeof(*out.ext1)));\n    out.ext1->{c_name} = (ivim_ts_{conv}_t*) calloc(1, sizeof(ivim_ts_{conv}_t));")
            text = text.replace(f"*out.{c_name}", f"*out.ext1->{c_name}")

    if name == "convertInternationalSignapplicablePeriod.h":
        for tag in ("year", "month_day", "hourMinutes"):
            text = text.replace(
                f"(struct ivim_ts_InternationalSign_applicablePeriod__{tag}*) calloc(1, sizeof(*out.{tag}))",
                f"static_cast<decltype(out.{tag})>(calloc(1, sizeof(*out.{tag})))",
            )

    if name in ("convertInternationalSigndistanceBetweenVehicles.h", "convertInternationalSignexemptedApplicablePeriod.h"):
        text = text.replace("ivim_ts_InternationalSign-", "ivim_ts_InternationalSign_")

    if name == "convertVehicleCharacteristicsRanges.h":
        text = text.replace("in.limits.numberOfAxles", "in.limits.choice.numberOfAxles")
        text = text.replace("out.limits.numberOfAxles", "out.limits.choice.numberOfAxles")
        text = text.replace("in.limits.limits.present", "in.limits.present")
        text = text.replace("in.limits.limits.choice.", "in.limits.choice.")
        text = text.replace("out.limits.limits.present", "out.limits.present")
        text = text.replace("out.limits.limits.choice.", "out.limits.choice.")
        text = text.replace("ivim_ts_VehicleCharacteristicsRanges__limits__limits_PR", "ivim_ts_VehicleCharacteristicsRanges__limits_PR")

    if name == "convertTrailerCharacteristicsList.h":
        text = text.replace(
            "toRos_TrailerCharacteristics(*(in.list.array[i]), el);",
            "toRos_TrailerCharacteristics(*reinterpret_cast<const ivim_ts_IVI_TrailerCharacteristics_t*>(in.list.array[i]), el);",
        )
        text = text.replace(
            "ivim_ts_IVI_TrailerCharacteristics_t* el = (ivim_ts_IVI_TrailerCharacteristics_t*) calloc(1, sizeof(ivim_ts_IVI_TrailerCharacteristics_t));",
            "auto* el = (struct ivim_ts_TrailerCharacteristics*) calloc(1, sizeof(ivim_ts_IVI_TrailerCharacteristics_t));",
        )
        text = text.replace(
            "toStruct_TrailerCharacteristics(in.array[i], *el);",
            "toStruct_TrailerCharacteristics(in.array[i], *reinterpret_cast<ivim_ts_IVI_TrailerCharacteristics_t*>(el));",
        )

    if name == "convertCompleteVehicleCharacteristics.h":
        text = text.replace(
            "toRos_TrainCharacteristics(*in.train, out.train);",
            "toRos_TrainCharacteristics(reinterpret_cast<const ivim_ts_TrainCharacteristics_t&>(*in.train), out.train);",
        )
        text = text.replace(
            "out.train = (ivim_ts_TrainCharacteristics_t*) calloc(1, sizeof(ivim_ts_TrainCharacteristics_t));",
            "out.train = (struct ivim_ts_TrainCharacteristics*) calloc(1, sizeof(ivim_ts_TrainCharacteristics_t));",
        )
        text = text.replace(
            "toStruct_TrainCharacteristics(in.train, *out.train);",
            "toStruct_TrainCharacteristics(in.train, reinterpret_cast<ivim_ts_TrainCharacteristics_t&>(*out.train));",
        )

    if name == "convertGddAttribute.h":
        text = re.sub(
            r'    break;case ivim_ts_GddAttribute_PR_ddd:\n'
            r'    toRos_InternationalSigndestinationInformation\(in\.choice\.ddd, out\.ddd\);\n'
            r'    out\.choice = ivim_ts_msgs::GddAttribute::CHOICE_DDD;\n',
            "",
            text,
        )
        text = re.sub(
            r'    break;case ivim_ts_msgs::GddAttribute::CHOICE_DDD:\n'
            r'    toStruct_InternationalSigndestinationInformation\(in\.ddd, out\.choice\.ddd\);\n'
            r'    out\.present = ivim_ts_GddAttribute_PR_ddd;\n',
            "",
            text,
        )

    if name == "convertDDDIOLIST.h":
        text = text.replace("ivim_ts_DDD-IO-LIST_t", "ivim_ts_DDD_IO_LIST_t")
        text = text.replace("sizeof(ivim_ts_DDD-IO-LIST_t)", "sizeof(ivim_ts_DDD_IO_LIST_t)")

    if name == "convertRSCode.h":
        text = text.replace("ivim_ts_RSCode__code__code_PR_", "ivim_ts_RSCode__code_PR_")
        text = text.replace("in.code.itisCodes", "in.code.choice.itisCodes")
        text = text.replace("out.code.itisCodes", "out.code.choice.itisCodes")

    if name == "convertInternationalSigndestinationInformation.h":
        text = text.replace(
            "  toRos_DDDIOLIST(in.ioList, out.io_list);\n",
            "  toRos_DDDIOLIST(*in.ioList, out.io_list);\n",
        )
        text = text.replace(
            "  toStruct_DDDIOLIST(in.io_list, out.ioList);\n",
            "  out.ioList = (ivim_ts_DDD_IO_LIST_t*) calloc(1, sizeof(ivim_ts_DDD_IO_LIST_t));\n"
            "  toStruct_DDDIOLIST(in.io_list, *out.ioList);\n",
        )

    if name == "convertDestinationPlace.h":
        text = text.replace(
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "    out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "    out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "    out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "  }\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n",
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "      out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "      out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "      out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "    }\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n"
            "  }\n",
        )
        text = text.replace(
            "etsi_its_primitives_conversion::toRos_OCTET_STRING(in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "    out.dest_rs_code_country_code_is_present = true;",
            "if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }",
        )
        text = text.replace(
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "    out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "    out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "    out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "  }\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n",
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "      out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "      out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "      out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "    }\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n"
            "  }\n",
        )
        text = text.replace(
            "etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.dest_rs_code_country_code, out.destRSCode->pictogramCode.countryCode);",
            "out.destRSCode->pictogramCode.countryCode = (OCTET_STRING_t*) calloc(1, sizeof(OCTET_STRING_t));\n"
            "    etsi_its_primitives_conversion::toStruct_OCTET_STRING(in.dest_rs_code_country_code, *out.destRSCode->pictogramCode.countryCode);",
        )
        text = text.replace(
            "ivim_ts_GddStructure_t__serviceCategoryCode_PR_",
            "ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_",
        )
        text = text.replace("out.service_category_code_choice", "out.dest_rs_code_service_category_code_choice")
        text = text.replace("in.service_category_code_choice", "in.dest_rs_code_service_category_code_choice")
        text = text.replace(
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "    out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "    out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "  }\n"
            "  if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "    out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "    out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "  }\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "  etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n",
            "  if (in.destRSCode) {\n"
            "    if (in.destRSCode->pictogramCode.countryCode) {\n"
            "      etsi_its_primitives_conversion::toRos_OCTET_STRING(*in.destRSCode->pictogramCode.countryCode, out.dest_rs_code_country_code);\n"
            "      out.dest_rs_code_country_code_is_present = true;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_trafficSignPictogram) {\n"
            "      out.dest_rs_code_service_category_code_traffic_sign_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.trafficSignPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_TRAFFIC_SIGN_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_publicFacilitiesPictogram) {\n"
            "      out.dest_rs_code_service_category_code_public_facilities_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.publicFacilitiesPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_PUBLIC_FACILITIES_PICTOGRAM;\n"
            "    }\n"
            "    if (in.destRSCode->pictogramCode.serviceCategoryCode.present == ivim_ts_GddStructure__pictogramCode__serviceCategoryCode_PR_ambientOrRoadConditionPictogram) {\n"
            "      out.dest_rs_code_service_category_code_ambient_or_road_condition_pictogram = in.destRSCode->pictogramCode.serviceCategoryCode.choice.ambientOrRoadConditionPictogram;\n"
            "      out.dest_rs_code_service_category_code_choice = ivim_ts_msgs::DestinationPlace::DEST_RS_CODE_CHOICE_SERVICE_CATEGORY_CODE_AMBIENT_OR_ROAD_CONDITION_PICTOGRAM;\n"
            "    }\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.nature, out.dest_rs_code_nature);\n"
            "    etsi_its_primitives_conversion::toRos_INTEGER(in.destRSCode->pictogramCode.pictogramCategoryCode.serialNumber, out.dest_rs_code_serial_number);\n"
            "  }\n",
        )

    text = text.replace("ivim_ts_InternationalSign_destinationInformation.h", "ivim_ts_InternationalSign-destinationInformation.h")
    text = text.replace("ivim_ts_InternationalSign_distanceBetweenVehicles.h", "ivim_ts_InternationalSign-distanceBetweenVehicles.h")
    text = text.replace("ivim_ts_InternationalSign_exemptedApplicablePeriod.h", "ivim_ts_InternationalSign-exemptedApplicablePeriod.h")

    return text


def postprocessIvimConversionHeaders(output_dir: str):
    changed = 0
    for header in Path(output_dir).glob("convert*.h"):
        original = header.read_text(encoding="utf-8")
        text = rewriteIvimConversionHeader(header.name, original)
        if header.name == "convertDestinationPlace.h" and not text.endswith("\n"):
            text += "\n"
        if text != original:
            header.write_text(text, encoding="utf-8")
            changed += 1
    print(f"Postprocessed {changed} IVIM conversion headers")


def main():

    args = parseCli()
    normalizeInputFiles(args)

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
    elif args.type == "denm_ts":
        msg_type = "DENM"
    elif args.type == "mapem_ts":
        msg_type = "MAPEM"
    elif args.type == "spatem_ts":
        msg_type = "SPATEM"
    elif args.type == "vam_ts":
        msg_type = "VAM"
    elif args.type == "ivim_ts":
        msg_type = "IVIM"
    elif args.type == "mcm_uulm":
        msg_type = "MCM"
    header_files = findDependenciesOfConversionHeaders(os.path.join(args.output_dir, f"convert{msg_type}.h"), args.type, [f"convert{msg_type}"])
    header_files += additionalMessageTypes(args.output_dir, msg_type)
    header_files = sortHeaderFiles(header_files)

    for f in glob.glob(os.path.join(args.output_dir, "*.h")):
        header_filename = os.path.splitext(os.path.basename(f))[0]
        if header_filename not in header_files:
            os.remove(f)

    if args.type == "ivim_ts":
        postprocessIvimConversionHeaders(args.output_dir)

    print(f"Generated {len(header_files)} conversion headers for {msg_type}")

if __name__ == "__main__":

    main()
