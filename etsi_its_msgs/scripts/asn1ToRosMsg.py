#!/usr/bin/env python

import argparse
import os
import re
from copy import deepcopy
from typing import Dict, List, Optional

import asn1tools


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


def resolveAsn1Type(asn1_type: Dict, asn1_types: Dict[str, Dict]) -> Dict:

    if asn1_type["type"] in asn1_types:
        type_type = resolveAsn1Type(asn1_types[asn1_type["type"]], asn1_types)
        type_type = asn1_type if type_type is None else type_type
        if "name" in asn1_type:
            type_type["name"] = asn1_type["name"]
        return type_type
    elif asn1_type["type"] in ASN1_PRIMITIVES_2_ROS:
        return asn1_type
    else:
        return None


def asn1TypeToRosMsgStr(asn1_type: Dict, asn1_types: Dict[str, Dict]) -> Optional[str]:

    ros_msg = ""

    if asn1_type["type"] in ("SEQUENCE", "CHOICE"):

        # flag for choices
        if asn1_type["type"] == "CHOICE":
            ros_msg += f"int32 type\n\n"

        # loop members
        for i_member, member in enumerate(asn1_type["members"]):

            member_lines = []
            member_comments = []

            if member is None:
                continue

            # resolve type aliases
            member = deepcopy(resolveAsn1Type(member, asn1_types))

            # resolve arrays/enumerations
            if member["type"] in asn1_types:
                member_info = asn1_types[member["type"]]

                # arrays
                if member_info["type"] == "SEQUENCE OF":
                    member_name = member["name"]
                    member = deepcopy(member_info)
                    member["name"] = member_name
                    member["type"] = member_info["element"]["type"] + "[]"
                    member_lines.append(f"{member['type']} {member['name']}")

                # enumerations to integer constants
                elif member_info["type"] == "ENUMERATED":
                    member_name = member["name"]
                    member = deepcopy(member_info)
                    member["name"] = member_name
                    member["type"] = "INTEGER"
                    member_lines.append(f"{member['type']} {member['name']}")
                    for val in member_info.get("values", {}):
                        if val is None:
                            continue
                        (k, v) = val
                        member_lines.append(f"INTEGER {camel2SNAKE(member['name'])}_{camel2SNAKE(k)} = {v}")
                
                else:
                    member_lines.append(f"{member['type']} {member['name']}")
            else:
                member_lines.append(f"{member['type']} {member['name']}")

            # constant for choice flag
            if asn1_type["type"] == "CHOICE":
                member_lines.append(f"INTEGER TYPE_{camel2SNAKE(member['name'])} = {i_member}")

            # named constants
            for k, v in member.get("named-numbers", {}).items():
                member_lines.append(f"{member['type']} {camel2SNAKE(member['name'])}_{camel2SNAKE(k)} = {v}")

            # remaining info as comments
            for k, v in member.items():
                if k not in ("type", "name", "values", "named-numbers", "element"):
                    member_comments.append(f"{k}: {v}")

            # replace ASN1 with ROS primitives
            for idx, line in enumerate(member_lines):
                for asn1_primitive in ASN1_PRIMITIVES_2_ROS:
                    if line.startswith(asn1_primitive):
                        line = line.replace(asn1_primitive, ASN1_PRIMITIVES_2_ROS[asn1_primitive], 1)
                member_lines[idx] = line

            # append to ROS message
            for line in member_comments:
                ros_msg += f"# {line}\n"
            for line in member_lines:
                ros_msg += f"{line}\n"
            ros_msg += "\n"

    elif asn1_type["type"] in ASN1_PRIMITIVES_2_ROS:

        # resolved as members
        return None

    elif asn1_type["type"] in asn1_types:

        # resolved as members
        return None

    elif asn1_type["type"] in ("ENUMERATED", "SEQUENCE OF"):

        # resolved as members
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
