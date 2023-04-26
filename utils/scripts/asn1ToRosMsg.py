#!/usr/bin/env python

import argparse
import os
import re
import warnings
from typing import Dict, List, Optional, Tuple

import asn1tools
import jinja2
import numpy as np


ASN1_PRIMITIVES_2_ROS = {
    "BOOLEAN": "bool",
    "INTEGER": "long",
    "IA5String": "string",
    "UTF8String": "string",
    "BIT STRING": "bool[]",
    "OCTET STRING": "string",
    "NumericString": "string",
    "VisibleString": "string",
}


TO_ROS = "toRos"
TO_STRUCT = "toStruct"


class CppFileStr:
    
    INDENT_PLACEHOLDER = "<INDENT>"
    INDENT = "  "
    
    # TODO: better to track lines and indentation per line?
    def __init__(self, s: str = "", indentation: int = 0):
        self.s = s
        self.indentation = indentation
    
    def __add__(self, other: str):
        br = "\n" if len(self.s) > 0 else ""
        if isinstance(other, CppFileStr):
            s = self.s + br + other.s
            indentation = other.indentation
        elif isinstance(other, str):
            delta_indentation = other.count("{") - other.count("}")
            indentation = self.indentation if delta_indentation >= 0 else self.indentation + delta_indentation
            indentation = max(indentation, 0)
            s = self.s + br
            s += indentation * self.INDENT_PLACEHOLDER + other
            indentation = max(self.indentation + delta_indentation, 0)
        return CppFileStr(s, indentation)

    def __str__(self) -> str:
        return self.s.replace(self.INDENT_PLACEHOLDER, self.INDENT)


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
                raise ValueError(f"Type '{type}' from '{doc}' is a duplicate")

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


def asn1TypesToFileStr(etsi_type: str, asn1_types: Dict[str, Dict]) -> Dict[str, str]:

    ros_msg_by_type = {}

    # loop all types
    for t_name, type in asn1_types.items():

        # ASN1 to ROS message
        ros_msg_by_type[t_name] = asn1TypeToJinjaContext(etsi_type, t_name, type, asn1_types)

    return ros_msg_by_type


def exportRosMsg(ros_msg_by_type: Dict[str, str], asn1_docs: Dict, asn1_raw: Dict[str, str], output_dir: str):

    # load jinja template
    template_dir = os.path.join(os.path.dirname(__file__), "templates")
    jinja_env = jinja2.Environment(loader=jinja2.FileSystemLoader(template_dir), trim_blocks=False)
    jinja_template = jinja_env.get_template("RosMessageType.msg")

    # loop over all types
    for type, jinja_context in ros_msg_by_type.items():

        if jinja_context is None:
            continue

        # add raw asn1 definition as comment to ROS .msg
        if type in asn1_raw:
            jinja_context["asn1_definition"] = asn1_raw[type].rstrip("\n")

        # create output directory
        doc_output_dir = os.path.join(output_dir, docForAsn1Type(type, asn1_docs))
        os.makedirs(doc_output_dir, exist_ok=True)

        # export ROS .msg using jinja template
        filename = os.path.join(doc_output_dir, f"{validRosType(type)}.msg")
        with open(filename, "w", encoding="utf-8") as file:
            file.write(jinja_template.render(jinja_context))
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


def asn1TypeToJinjaContext(etsi_type: str, t_name: str, asn1: Dict, asn1_types: Dict[str, Dict]) -> Dict:

    # TODO: remove msg
    msg = ""

    type = asn1["type"]
    
    context = {
        "asn1_definition": None,
        "comments": [],
        "etsi_type": None,
        "members": [],
        "t_name": None,
        "t_type": None,
    }

    # extra information (e.g. optional) as comments
    for k, v in asn1.items():
        if k not in ("type", "name", "members", "values", "element", "named-bits", "named-numbers", "optional"):
            context["comments"].append(f"{k}: {v}")

    # primitives
    if type in ASN1_PRIMITIVES_2_ROS:

        # resolve ROS msg type
        ros_type = ASN1_PRIMITIVES_2_ROS[type]
        name = asn1["name"] if "name" in asn1 else "value"
        
        # add fixed array size to BIT STRING boolean arrays
        if type == "BIT STRING" and "size" in asn1:
            if not isinstance(asn1["size"][0], tuple):
                ros_type = f"bool[{asn1['size'][0]}]"

        # choose simplest possible integer type
        if "restricted-to" in asn1 and type == "INTEGER":
            min_value = asn1["restricted-to"][0][0]
            max_value = asn1["restricted-to"][0][1]
            ros_type = simplestRosIntegerType(min_value, max_value)

        # parse member to jinja context
        member_context = {
            "type": ros_type,
            "name": validRosField(name),
            "constants": [],
        }

        # add constants for named numbers
        if "named-numbers" in asn1:
            for k, v in asn1["named-numbers"].items():
                constant_name = f"{camel2SNAKE(k)}"
                if "name" in asn1:
                    constant_name = f"{camel2SNAKE(asn1['name'])}_{constant_name}"
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(constant_name),
                    "value": v
                })

        # add index constants for named bits
        if "named-bits" in asn1:
            for k, v in asn1["named-bits"]:
                constant_name = f"{camel2SNAKE(k)}"
                member_context["constants"].append({
                    "type": "uint8",
                    "name": f"INDEX_{validRosField(constant_name)}",
                    "value": v
                })

        context["members"].append(member_context)

    # nested types
    elif type == "SEQUENCE":

        # recursively add all members
        for member in asn1["members"]:
            if member is None:
                continue
            member_context = asn1TypeToJinjaContext(etsi_type, t_name, member, asn1_types)
            if "optional" in member:
                member_context["members"][0]["optional"] = True
            context["members"].extend(member_context["members"])

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
                # member_msg = asn1TypeToRosMsgStr(etsi_type, t_name, member, asn1_types)
                member_msg = member_msg.split()[0] + f" {asn1['name']}_{member_msg.split()[1]}\n"
                msg += member_msg
                name = f"{camel2SNAKE(asn1['name'])}_{name}"
            # else:
                # msg += asn1TypeToRosMsgStr(etsi_type, t_name, member, asn1_types)
            msg += f"uint8 {name} = {im}"
            msg += "\n"

    # arrays
    elif type == "SEQUENCE OF":

        array_name = asn1["name"] if "name" in asn1 else "array"
        array_type = asn1['element']['type']
        element_name = array_type[0].lower() + array_type[1:]
        msg += f"{array_type}[] {array_name}"
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
        context["members"].append({
            "type": validRosType(type),
            "name": validRosField(name),
        })

    elif type == "NULL":

        pass
        
    else:

        warnings.warn(f"Cannot handle type '{type}'")

    msg = msg.rstrip("\n") + "\n"

    return context


def main():

    args = parseCli()

    asn1_docs, asn1_raw = parseAsn1Files(args.files)

    asn1_types = extractAsn1TypesFromDocs(asn1_docs)

    checkTypeMembersInAsn1(asn1_types)

    ros_msg_by_type = asn1TypesToFileStr(args.type, asn1_types)

    exportRosMsg(ros_msg_by_type, asn1_docs, asn1_raw, args.output_dir)


if __name__ == "__main__":
    
    main()
