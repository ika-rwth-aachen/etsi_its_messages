#!/usr/bin/env python

import argparse
import os
import re
import warnings
from typing import Dict, List, Optional, Tuple

import asn1tools
import numpy as np


ASN1_PRIMITIVES_2_ROS = {
    "BOOLEAN": "bool",
    "INTEGER": "int32",
    "IA5String": "string",
    "UTF8String": "string",
    "BIT STRING": "string",
    "OCTET STRING": "string",
    "NumericString": "string",
    "VisibleString": "string",
}


def parseCli():

    parser = argparse.ArgumentParser(
        description="Creates ROS .msg files from ASN1 definitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN1 type")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory")

    args = parser.parse_args()

    return args


def camel2SNAKE(s: str) -> str:

    return re.sub("([A-Z0-9])", r"_\1", s).upper().lstrip("_").replace("-", "")


def validRosType(s: str) -> str:

    return s.replace("-", "_")


def validRosField(s: str) -> str:

    return s.replace("-", "_")

def noSpace(s: str) -> str:
    
    return s.replace(" ", "_")


def parseAsn1Files(files: List[str]) -> Tuple[Dict, Dict[str, str]]:

    asn1_raw = {}
    for file in files:
        with open(file) as f:
            lines = f.readlines()
        raw_def = None
        for line in lines:
            if "::=" in line:
                if "{" in line:
                    type = line.split("::=")[0].strip()
                    raw_def = ""
                elif len(line.split("::=")) == 2:
                    type = line.split("::=")[0].strip()
                    raw_def = line
                    asn1_raw[type] = raw_def
                    raw_def = None
            if raw_def is not None:
                raw_def += line
                if "}" in line:
                    asn1_raw[type] = raw_def
                    raw_def = None

    asn1_docs = asn1tools.parse_files(files)

    return asn1_docs, asn1_raw


def docForAsn1Type(asn1_type: str, asn1_docs: Dict) -> Optional[str]:

    for doc, asn1 in asn1_docs.items():
        if asn1_type in asn1["types"]:
            return doc
    
    return None


def extractAsn1TypesFromDocs(asn1_docs: Dict) -> Dict[str, Dict]:

    asn1_types = {}
    for doc, asn1 in asn1_docs.items():
        for type in asn1["types"]:
            if type not in asn1_types:
                asn1_types[type] = asn1["types"][type]
            else:
                warnings.warn(f"Type '{type}' from '{doc}' is a duplicate")

    return asn1_types


def checkTypeMembersInAsn1(asn1_types: Dict[str, Dict]):

    known_types = ["SEQUENCE", "SEQUENCE OF", "CHOICE", "ENUMERATED", "NULL"]
    known_types += list(asn1_types.keys())
    known_types += list(ASN1_PRIMITIVES_2_ROS.keys())

    # loop all types
    for t_name, type in asn1_types.items():

        # loop all members in type
        for member in type.get("members", []):

            if member is None:
                continue

            # check if type is known
            if member["type"] not in known_types:
                if ".&" in member["type"]:
                    warnings.warn(
                        f"Type '{member['type']}' of member '{member['name']}' "
                        f"in '{t_name}' seems to relate to a 'CLASS' type, not "
                        f"yet supported")
                else:
                    raise TypeError(
                        f"Type '{member['type']}' of member '{member['name']}' "
                        f"in '{t_name}' is undefined")


def asn1TypesToFileStr(etsi_type: str, asn1_types: Dict[str, Dict]) -> List[Dict[str, str]]:

    ros_msg_by_type = {}
    converter_by_type = {}

    # loop all types
    for t_name, type in asn1_types.items():

        # ASN1 to ROS message and Converter
        ros_msg_by_type[t_name], converter_by_type[t_name] = asn1TypeToRosMsgStr(etsi_type, t_name, type, asn1_types)

    return ros_msg_by_type, converter_by_type

def exportConverterFiles(converter_by_type: Dict[str, str], output_dir: str):

    # loop over all types
    for type, h_str in converter_by_type.items():

        if h_str is None:
            continue

        # create output directory
        os.makedirs(output_dir, exist_ok=True)

        # export ROS .msg
        filename = os.path.join(output_dir, f"convert{type}.h")
        with open(filename, "w", encoding="utf-8") as file:
            file.write(h_str)
        print(filename)

def exportRosMsg(ros_msg_by_type: Dict[str, str], asn1_docs: Dict, asn1_raw: Dict[str, str], output_dir: str):

    # loop over all types
    for type, ros_msg in ros_msg_by_type.items():

        if ros_msg is None:
            continue

        # add raw asn1 definition as comment to ROS .msg
        if type in asn1_raw:
            raw_lines = [f"# {l}" for l in asn1_raw[type].split("\n")]
            raw = "# " + "-"*78 + "\n" + "\n".join(raw_lines) + "-"*78 + "\n"
            ros_msg = raw + "\n" + ros_msg

        # create output directory
        doc_output_dir = os.path.join(output_dir, docForAsn1Type(type, asn1_docs))
        os.makedirs(doc_output_dir, exist_ok=True)

        # export ROS .msg
        filename = os.path.join(doc_output_dir, f"{validRosType(type)}.msg")
        with open(filename, "w", encoding="utf-8") as file:
            file.write(ros_msg)
        print(filename)


def simplestRosIntegerType(min_value, max_value):

    if min_value >= np.iinfo(np.uint8).min and max_value <= np.iinfo(np.uint8).max:
        return "int64"
    elif min_value >= np.iinfo(np.uint16).min and max_value <= np.iinfo(np.uint16).max:
        return "int64"
    elif min_value >= np.iinfo(np.uint32).min and max_value <= np.iinfo(np.uint32).max:
        return "int64"
    elif min_value >= np.iinfo(np.uint64).min and max_value <= np.iinfo(np.uint64).max:
        return "int64"
    elif min_value >= np.iinfo(np.int8).min and max_value <= np.iinfo(np.int8).max:
        return "int64"
    elif min_value >= np.iinfo(np.int16).min and max_value <= np.iinfo(np.int16).max:
        return "int64"
    elif min_value >= np.iinfo(np.int32).min and max_value <= np.iinfo(np.int32).max:
        return "int64"
    elif min_value >= np.iinfo(np.int64).min and max_value <= np.iinfo(np.int64).max:
        return "int64"
    else:
        return ValueError(f"No ROS integer type supports range [{min_value}, {max_value}]")

def convertToPrimitiv(type, t_name, name) -> List[str]:
    if type == "IA5String" or type == "UTF8String" or type == "NumericString" or type == "VisibleString":
        memberType = "OCTET_STRING"
    else:
        memberType = noSpace(type)

    c_name = ""
    if name != "value":
        c_name = f".{name}"
    
    if memberType == "INTEGER":
        c2ros = f"\t\tconvert_toRos(_{t_name}_in{c_name}, {t_name}_out.{name});\n"
        ros2c = f"\t\tconvert_toC(_{t_name}_in.{name}, {t_name}_out{c_name});\n"
    else:
        c2ros = f"\t\tconvert_{memberType}toRos(_{t_name}_in{c_name}, {t_name}_out.{name});\n"
        ros2c = f"\t\tconvert_{memberType}toC(_{t_name}_in.{name}, {t_name}_out{c_name});\n"
    
    # if memberType == "INTEGER":
    #     c2ros = f"\t\tconvert_toRos(_{t_name}_in{c_name}, {t_name}_out.{name});\n"
    #     ros2c = f"\t\tconvert_toC(_{t_name}_in.{name}, {t_name}_out{c_name});\n"
    # else:
    #     c2ros = f"\t\t{t_name}_out.{name} = convert_toRos(_{t_name}_in{c_name});\n"
    #     ros2c = f"\t\t{t_name}_out{c_name} = convert_toC(_{t_name}_in.{name});\n"

    return c2ros, ros2c

def asn1TypeToRosMsgStr(etsi_type: str, t_name: str, asn1: Dict, asn1_types: Dict[str, Dict]) -> Optional[List[str]]:
    ns_msgs = f"etsi_its_{etsi_type}_msgs"
    msg = ""
    type = asn1["type"]
    header = f"#pragma once\n\n"
    header += f"#include <{t_name}.h>\n#include <{ns_msgs}/{t_name}.h>\n"
    namespace = f"namespace etsi_its_{etsi_type}_conversion\n"
    namespace += "{\n"
    c2ros = f"\t{ns_msgs}::{t_name} convert_{t_name}toRos(const {t_name}_t& _{t_name}_in)\n"
    ros2c = f"\t{t_name}_t convert_{t_name}toC(const {ns_msgs}::{t_name}& _{t_name}_in)\n"
    c2ros += "\t{\n"
    ros2c += "\t{\n"
    c2ros += f"\t\t{ns_msgs}::{t_name} {t_name}_out;\n"
    ros2c += f"\t\t{t_name}_t {t_name}_out;\n"

    # extra information (e.g. optional) as comments
    for k, v in asn1.items():
        if k not in ("type", "name", "members", "values", "element", "named-numbers"):
            msg += f"# {k}: {v}"
            msg += "\n"

    # primitives
    if type in ASN1_PRIMITIVES_2_ROS:

        # resolve ROS msg type
        ros_type = ASN1_PRIMITIVES_2_ROS[type]
        name = asn1["name"] if "name" in asn1 else "value"

        # choose simplest possible integer type
        if "restricted-to" in asn1 and type == "INTEGER":
            min_value = asn1["restricted-to"][0][0]
            max_value = asn1["restricted-to"][0][1]
            ros_type = simplestRosIntegerType(min_value, max_value)

        # add constants for named numbers
        if "named-numbers" in asn1:
            for k, v in asn1["named-numbers"].items():
                constant_name = f"{camel2SNAKE(k)}"
                if "name" in asn1:
                    constant_name = f"{camel2SNAKE(asn1['name'])}_{constant_name}"
                msg += f"{ros_type} {validRosField(constant_name)} = {v}"
                msg += "\n"

        msg += f"{ros_type} {validRosField(name)}"
        msg += "\n"

        # Converter
        if f"convert{noSpace(type)}.h" not in header:
            header += f"#include <convert{noSpace(type)}.h>\n"
        c2ros += convertToPrimitiv(type,t_name,name)[0]
        ros2c += convertToPrimitiv(type,t_name,name)[1]

    # nested types
    elif type == "SEQUENCE":

        # recursively add all members
        for member in asn1["members"]:
            if member is None:
                continue
            msg += asn1TypeToRosMsgStr(etsi_type, t_name, member, asn1_types)[0]
            msg += "\n"

            # Converter
            memberName = member["name"]
            memberType = member["type"]
            if f"convert{memberType}.h" not in header:    
                header += f"#include <convert{memberType}.h>\n"
            if "optional" in member.keys():
                c2ros+=f"\t\tif(_{t_name}_in.{memberName})\n"
                ros2c+=f"\t\tif(XY.msg)\n"
                c2ros+="\t\t{\n"
                ros2c+="\t\t{\n"
                c2ros += f"\t\t\t{t_name}_out.{memberName} = convert_{memberType}toRos(*_{t_name}_in.{memberName});\n"
                ros2c += f"\t\t\t{t_name}_out.{memberName} = *convert_{memberType}toC(_{t_name}_in.{memberName});\n"
                c2ros+="\t\t}\n"
                ros2c+="\t\t}\n"
            else:
                if (memberType in ASN1_PRIMITIVES_2_ROS):
                    c2ros += convertToPrimitiv(memberType,t_name,memberName)[0]
                    ros2c += convertToPrimitiv(memberType,t_name,memberName)[1]
                else:
                    c2ros += f"\t\t{t_name}_out.{memberName} = convert_{memberType}toRos(_{t_name}_in.{memberName});\n"
                    ros2c += f"\t\t{t_name}_out.{memberName} = convert_{memberType}toC(_{t_name}_in.{memberName});\n"

    # type aliases with multiple options
    elif type == "CHOICE":

        # add flag for indicating active option
        name = "choice"
        if "name" in asn1:
            name = f"{asn1['name']}_{name}"
        msg += f"uint8 {name}"
        msg += "\n"
        msg += "\n"

        # recursively add members for all options, incl. constant for flag
        for im, member in enumerate(asn1["members"]):
            if member is None:
                continue
            name = f"CHOICE_{camel2SNAKE(member['name'])}"
            if "name" in asn1:
                member_msg = asn1TypeToRosMsgStr(etsi_type, t_name, member, asn1_types)[0]
                member_msg = member_msg.split()[0] + f" {asn1['name']}_{member_msg.split()[1]}\n"
                msg += member_msg
                name = f"{camel2SNAKE(asn1['name'])}_{name}"
            else:
                msg += asn1TypeToRosMsgStr(etsi_type, t_name, member, asn1_types)[0]
            msg += f"uint8 {name} = {im}"
            msg += "\n"

            # Converter
            memberName = member["name"]
            memberType = member["type"]
            if f"convert{memberType}.h" not in header:
                header += f"#include <convert{memberType}.h>\n"
            c2ros += f"\t\tif(_{t_name}_in.present == {t_name}_PR::{t_name}_PR_{memberName})\n"
            c2ros += "\t\t{\n"
            c2ros += f"\t\t\t{t_name}_out.{memberName} = convert_{memberType}toRos(_{t_name}_in.choice.{memberName});\n"
            c2ros += f"\t\t\t{t_name}_out.choice = {ns_msgs}::{t_name}::{name};\n"
            c2ros += "\t\t}\n"

            ros2c += f"\t\tif(_{t_name}_in.choice == {ns_msgs}::{t_name}::{name})\n"
            ros2c += "\t\t{\n"
            ros2c += f"\t\t\t{t_name}_out.choice.{memberName} = convert_{memberType}toC(_{t_name}_in.{memberName});\n"
            ros2c += f"\t\t\t{t_name}_out.present = {t_name}_PR::{t_name}_PR_{memberName};\n"
            ros2c += "\t\t}\n"

    # arrays
    elif type == "SEQUENCE OF":

        msg += f"{asn1['element']['type']}[] array"
        msg += "\n"

    # enums
    elif type == "ENUMERATED":

        # choose simplest possible integer type
        values = [val[1] for val in asn1["values"] if val is not None]
        min_value = min(values)
        max_value = max(values)
        ros_type = simplestRosIntegerType(min_value, max_value)

        # add constants for all values
        for val in asn1["values"]:
            if val is None:
                continue
            msg += f"{ros_type} {camel2SNAKE(val[0])} = {val[1]}"
            msg += "\n"

        # add field for active value
        msg += f"{ros_type} value"
        msg += "\n"

        # Converter
        name = asn1["name"] if "name" in asn1 else "value"
        c2ros += f"\t\t{t_name}_out.{name} = _{t_name}_in;\n"
        ros2c += f"\t\t{t_name}_out = _{t_name}_in.{name};\n"

    # custom types
    elif type in asn1_types:

        name = asn1["name"] if "name" in asn1 else "value"
        msg += f"{validRosType(type)} {validRosField(name)}"
        msg += "\n"

    elif type == "NULL":

        pass
        
    else:

        warnings.warn(f"Cannot handle type '{type}'")

    msg = msg.rstrip("\n") + "\n"

    return_value = f"\t\treturn {t_name}_out;\n"
    return_value += "\t}\n"

    converter_str = header + "\n" + namespace + c2ros + return_value + "}"

    return msg, converter_str


def main():

    args = parseCli()

    asn1_docs, asn1_raw = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    ros_msg_by_type, converter_by_type = asn1TypesToFileStr(args.type, asn1_types)

    exportRosMsg(ros_msg_by_type, asn1_docs, asn1_raw, args.output_dir)

    exportConverterFiles(converter_by_type, "/home/jpbusch/git/ros_etsi_its_messages/etsi_its_conversion/include")

if __name__ == "__main__":
    
    main()
