#!/usr/bin/env python

import argparse
import os
import re
from typing import Dict, List, Optional

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
}


def parseCli():

    parser = argparse.ArgumentParser(
        description="Creates ROS .msg files from ASN1 definitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-o", "--output-dir", type=str, default=os.path.realpath(os.path.join(os.path.dirname(__file__), os.pardir, "msg")), help="output directory")

    args = parser.parse_args()

    return args


def camel2SNAKE(s: str) -> str:

    return re.sub("([A-Z0-9])", r"_\1", s).upper().lstrip("_").replace("-", "")


def parseAsn1Files(files: List[str]) -> Dict:

    return asn1tools.parse_files(files)


def extractAsn1TypesFromDocs(asn1_docs: Dict) -> Dict[str, Dict]:

    asn1_types = {}
    for doc, asn1 in asn1_docs.items():
        asn1_types.update(asn1["types"])

    return asn1_types


def checkTypeMembersInAsn1(asn1_types: Dict[str, Dict]):

    known_types = list(asn1_types.keys())

    # loop all types
    for t_name, type in asn1_types.items():

        # loop all members in type
        for member in type.get("members", []):

            if member is None:
                continue

            # check if type is known
            if member["type"] not in known_types and member["type"] not in ASN1_PRIMITIVES_2_ROS:
                raise TypeError(f"Member '{member['name']}' of type '{member['type']}' in '{t_name}' is undefined")


def asn1TypesToRosMsgStr(asn1_types: Dict[str, Dict]) -> Dict[str, str]:

    ros_msg_by_type = {}

    # loop all types
    for t_name, type in asn1_types.items():

        # ASN1 to ROS message
        ros_msg_by_type[t_name] = asn1TypeToRosMsgStr(type, asn1_types)

    return ros_msg_by_type


def exportRosMsg(ros_msg_by_type: Dict[str, str], output_dir):

    # create output directory
    os.makedirs(output_dir, exist_ok=True)

    # loop over all types
    for type, ros_msg in ros_msg_by_type.items():

        if ros_msg is None:
            continue

        # export ROS .msg
        filename = os.path.join(output_dir, f"{type}.msg")
        with open(filename, "w", encoding="utf-8") as file:
            file.write(ros_msg)
        print(filename)


def simplestRosIntegerType(min_value, max_value):

    if min_value >= np.iinfo(np.uint8).min and max_value <= np.iinfo(np.uint8).max:
        return "uint8"
    elif min_value >= np.iinfo(np.uint16).min and max_value <= np.iinfo(np.uint16).max:
        return "uint16"
    elif min_value >= np.iinfo(np.uint32).min and max_value <= np.iinfo(np.uint32).max:
        return "uint32"
    elif min_value >= np.iinfo(np.uint64).min and max_value <= np.iinfo(np.uint64).max:
        return "uint64"
    elif min_value >= np.iinfo(np.int8).min and max_value <= np.iinfo(np.int8).max:
        return "int8"
    elif min_value >= np.iinfo(np.int16).min and max_value <= np.iinfo(np.int16).max:
        return "int16"
    elif min_value >= np.iinfo(np.int32).min and max_value <= np.iinfo(np.int32).max:
        return "int32"
    elif min_value >= np.iinfo(np.int64).min and max_value <= np.iinfo(np.int64).max:
        return "int64"
    else:
        return ValueError(f"No ROS integer type supports range [{min_value}, {max_value}]")


def asn1TypeToRosMsgStr(asn1: Dict, asn1_types: Dict[str, Dict]) -> Optional[str]:

    msg = ""
    type = asn1["type"]

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
                msg += f"{ros_type} {constant_name} = {v}"
                msg += "\n"

        msg += f"{ros_type} {name}"
        msg += "\n"

    # nested types
    elif type == "SEQUENCE":

        # recursively add all members
        for member in asn1["members"]:
            if member is None:
                continue
            msg += asn1TypeToRosMsgStr(member, asn1_types)
            msg += "\n"

    # type aliases with multiple options
    elif type == "CHOICE":

        # add flag for indicating active option
        msg += "uint8 choice"
        msg += "\n"
        msg += "\n"

        # recursively add members for all options, incl. constant for flag
        for im, member in enumerate(asn1["members"]):
            if member is None:
                continue
            msg += asn1TypeToRosMsgStr(member, asn1_types)
            msg += f"uint8 CHOICE_{camel2SNAKE(member['name'])} = {im}"
            msg += "\n"

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

    # custom types
    elif type in asn1_types:

        name = asn1["name"] if "name" in asn1 else "value"
        msg += f"{asn1['type']} {name}"
        msg += "\n"

    else:

        raise NotImplementedError(f"Cannot handle type '{type}'")

    return msg


def main():

    args = parseCli()

    asn1_docs = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    ros_msg_by_type = asn1TypesToRosMsgStr(asn1_types)

    exportRosMsg(ros_msg_by_type, args.output_dir)


if __name__ == "__main__":
    
    main()
