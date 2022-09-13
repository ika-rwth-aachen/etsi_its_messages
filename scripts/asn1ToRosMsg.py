#!/usr/bin/env python

import argparse
import os
import re
from typing import Dict, List, Optional

import asn1tools

# TODO:
# BIT STRING
# ENUMERATE
# SEQUENCE OF
# ProtectedZoneID
# NumericString
# OCTET STRING

ASN1_PRIMITIVE_TYPES = [
    "BOOLEAN",
    "INTEGER",
    "IA5String",
    "UTF8String",
]

ASN1_ARRAY_TYPES = [
    "SEQUENCE",
]


def parseCli():

    parser = argparse.ArgumentParser(
        description="Creates ROS .msg files from ASN1 definitions.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-o", "--output-dir", type=str, default=os.path.realpath(os.path.join(os.path.dirname(__file__), os.pardir, "msg")), help="output directory")

    args = parser.parse_args()

    return args


def camel2SNAKE(s: str) -> str:

    return re.sub("([A-Z0-9])", r"_\1", s).upper().lstrip("_")


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
            if member["type"] not in known_types and member["type"] not in ASN1_PRIMITIVE_TYPES:
                raise TypeError(f"Member '{member['name']}' of type '{member['type']}' in '{t_name}' is undefined")


def asn1TypesToRosMsgStr(asn1_types: Dict[str, Dict]) -> Dict[str, str]:

    ros_msg_by_type = {}

    # loop all types
    for t_name, type in asn1_types.items():

        # ASN1 to ROS message
        ros_msg_by_type[t_name] = asn1TypeToRosMsgStr(type)

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


def asn1TypeToRosMsgStr(asn1_type: Dict) -> Optional[str]:

    ros_msg = ""

    type_type = asn1_type["type"]

    if type_type in ("SEQUENCE", "CHOICE"):

        # flag for choices
        if type_type == "CHOICE":
            ros_msg += f"INTEGER type\n\n"

        for i_member, member in enumerate(asn1_type["members"]):

            member_lines = []
            member_comments = []

            if member is None:
                continue

            member_lines.append(f"{member['type']} {member['name']}")

            # constant for choice flag
            if type_type == "CHOICE":
                member_lines.append(f"INTEGER TYPE_{camel2SNAKE(member['name'])} = {i_member}")

            # named constants
            for k, v in member.get("named-numbers", {}).items():
                member_lines.append(f"{member['type']} {camel2SNAKE(member['name'])}_{camel2SNAKE(k)} = {v}")

            # remaining info as comments
            for k, v in member.items():
                if k not in ("type", "name", "values", "named-numbers", "element"):
                    member_comments.append(f"{k}: {v}")

            # append to ROS message
            for line in member_comments:
                ros_msg += f"# {line}\n"
            for line in member_lines:
                ros_msg += f"{line}\n"
            ros_msg += "\n"

    elif type_type in ASN1_PRIMITIVE_TYPES:

        return None

    else:

        raise NotImplementedError

    return ros_msg


def main():

    args = parseCli()

    asn1_docs = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    ros_msg_by_type = asn1TypesToRosMsgStr(asn1_types)

    exportRosMsg(ros_msg_by_type, args.output_dir)


if __name__ == "__main__":
    
    main()
