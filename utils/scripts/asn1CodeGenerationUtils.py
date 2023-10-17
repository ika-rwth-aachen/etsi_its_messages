#!/usr/bin/env python

import re
import warnings
from typing import Dict, List, Optional, Tuple

import asn1tools
import numpy as np


ASN1_PRIMITIVES_2_ROS = {
    "BOOLEAN": "bool",
    "INTEGER": "long",
    "IA5String": "string",
    "UTF8String": "string",
    "BIT STRING": "uint8[]",
    "OCTET STRING": "uint8[]",
    "NumericString": "string",
    "VisibleString": "string",
}


def camel2SNAKE(s: str) -> str:
    """Converts a camelCase string to SNAKE_CASE.

    Args:
        s (str): camelCaseString

    Returns:
        str: SNAKE_CASE_STRING
    """

    return re.sub("([A-Z0-9])", r"_\1", s).upper().lstrip("_").replace("-", "")

def camel2snake(s: str) -> str:
    """Converts a camelCase string to snake_case.

    Args:
        s (str): camelCaseString

    Returns:
        str: snake_case_string
    """
    s = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', s)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s).lower()

def validRosType(s: str) -> str:
    """Converts a string to make it a valid ROS message type.

    Args:
        s (str): Not-A-Message-Type

    Returns:
        str: A_Message_Type
    """

    return s.replace("-", "_")


def validRosField(s: str) -> str:
    """Converts a string to make it a valid ROS message field name.

    Args:
        s (str): Not-A-Ros-Message-Field

    Returns:
        str: A_Ros_Message_Field
    """

    return s.replace("-", "_")


def noSpace(s: str) -> str:
    """Replaces any spaces in a string with underscores.

    Args:
        s (str): my string

    Returns:
        str: my_string
    """

    return s.replace(" ", "_")


def simplestRosIntegerType(min_value: int, max_value: int) -> str:
    """Returns the simplest/smallest ROS integer type covering the specified range.

    Args:
        min_value (int): minimum value
        max_value (int): maximum value

    Raises:
        ValueError: if specified range is not supported by any ROS integer type

    Returns:
        str: simplest/smallest ROS integer type, e.g., `uint32`
    """

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


def parseAsn1Files(files: List[str]) -> Tuple[Dict, Dict[str, str]]:
    """Parses ASN1 files.

    Args:
        files (List[str]): filepaths

    Returns:
        Tuple[Dict, Dict[str, str]]: parsed type information by document, raw string definition by type
    """

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
    """Finds the ASN1 document where a specific type is defined.

    Args:
        asn1_type (str): type name
        asn1_docs (Dict): parsed type information by document (from `parseAsn1Files`)

    Returns:
        Optional[str]: document name where type is defined, `None` if not found
    """

    for doc, asn1 in asn1_docs.items():
        if asn1_type in asn1["types"]:
            return doc

    return None


def extractAsn1TypesFromDocs(asn1_docs: Dict) -> Dict[str, Dict]:
    """Extracts all parsed ASN1 type information from multiple ASN1 documents.

    Args:
        asn1_docs (Dict): type information by document

    Raises:
        ValueError: if a type is found in multiple documents

    Returns:
        Dict[str, Dict]: type information by type
    """

    asn1_types = {}
    for doc, asn1 in asn1_docs.items():
        for type in asn1["types"]:
            if type not in asn1_types:
                asn1_types[type] = asn1["types"][type]
            else:
                raise ValueError(f"Type '{type}' from '{doc}' is a duplicate")

    return asn1_types

def extractAsn1ValuesFromDocs(asn1_docs: Dict) -> Dict[str, Dict]:
    """Extracts all parsed ASN1 type information from multiple ASN1 documents.

    Args:
        asn1_docs (Dict): type information by document

    Raises:
        ValueError: if a type is found in multiple documents

    Returns:
        Dict[str, Dict]: type information by type
    """

    asn1_values = {}
    for doc, asn1 in asn1_docs.items():
        for value in asn1["values"]:
            if value not in asn1_values:
                asn1_values[value] = asn1["values"][value]
            else:
                raise ValueError(f"Value '{value}' from '{doc}' is a duplicate")

    return asn1_values


def checkTypeMembersInAsn1(asn1_types: Dict[str, Dict]):
    """Checks if all type information is known and supported.

    This helps to check whether the types of all members of a type are also known.

    Args:
        asn1_types (Dict[str, Dict]): type information by type

    Raises:
        TypeError: if the type of a member is not part of the given types, hence unknown
    """

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


def asn1TypeToJinjaContext(t_name: str, asn1: Dict, asn1_types: Dict[str, Dict], asn1_values: Dict[str, Dict]) -> Dict:
    """Builds a jinja context containing all type information required to fill the templates / code generation.

    Args:
        t_name (str): type name
        asn1 (Dict): type information
        asn1_types (Dict[str, Dict]): type information of all types by type

    Returns:
        Dict: jinja context
    """

    type = asn1["type"]

    context = {
        "asn1_definition": None,
        "comments": [],
        "etsi_type": None,
        "members": [],
        "t_name": t_name,
        "t_name_snake": camel2snake(t_name),
        "type": noSpace(type),
        "asn1_type": type,
        "is_primitive": False,
    }

    # extra information / asn1 fields that are not processed as comments
    for k, v in asn1.items():
        if k not in ("type", "element", "members", "name", "named-bits", "named-numbers", "optional", "restricted-to", "size", "values", "default"):
            context["comments"].append(f"{k}: {v}")

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

        # parse member to jinja context
        member_context = {
            "type": ros_type,
            "asn1_type": type,
            "name": camel2snake(validRosField(name)),
            "name_cc": validRosField(name),
            "constants": [],
            "is_primitive": True,
            "has_bits_unused": False,
        }

        # add bits_unused field for BIT STRINGs
        if type == "BIT STRING":
            member_context["has_bits_unused"] = True

        # add constants for limits
        if "restricted-to" in asn1:
            min_value = asn1["restricted-to"][0][0]
            max_value = asn1["restricted-to"][0][1]
            min_constant_name = "MIN"
            max_constant_name = "MAX"
            if "name" in asn1:
                min_constant_name = f"{camel2SNAKE(asn1['name'])}_{min_constant_name}"
                max_constant_name = f"{camel2SNAKE(asn1['name'])}_{max_constant_name}"
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(min_constant_name),
                "value": min_value
            })
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(max_constant_name),
                "value": max_value
            })

        # add constants for size limits
        if "size" in asn1:
            if isinstance(asn1["size"][0], tuple):
                min_size = asn1["size"][0][0]
                max_size = asn1["size"][0][1]
                ros_type = simplestRosIntegerType(min_size, max_size)
                min_size_constant_name = "MIN_SIZE" if type != "BIT STRING" else "MIN_SIZE_BITS"
                max_size_constant_name = "MAX_SIZE" if type != "BIT STRING" else "MAX_SIZE_BITS"
                if "name" in asn1:
                    min_size_constant_name = f"{camel2SNAKE(asn1['name'])}_{min_size_constant_name}"
                    max_size_constant_name = f"{camel2SNAKE(asn1['name'])}_{max_size_constant_name}"
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(min_size_constant_name),
                    "value": min_size
                })
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(max_size_constant_name),
                    "value": max_size
                })
            else:
                size = asn1["size"][0]
                ros_type = simplestRosIntegerType(size, size)
                size_constant_name = "SIZE" if type != "BIT STRING" else "SIZE_BITS"
                if "name" in asn1:
                    size_constant_name = f"{camel2SNAKE(asn1['name'])}_{size_constant_name}"
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(size_constant_name),
                    "value": size
                })

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
                    "name": f"BIT_INDEX_{validRosField(constant_name)}",
                    "value": v
                })

        context["members"].append(member_context)

    # nested types
    elif type == "SEQUENCE":

        # recursively add all members
        for member in asn1["members"]:
            if member is None:
                continue
            member_context = asn1TypeToJinjaContext(t_name, member, asn1_types, asn1_values)
            if "optional" in member:
                member_context["members"][0]["optional"] = True
            if "default" in member:
                member_context["members"][0]["default"] = True
                if member["default"] in asn1_values:
                    defaultValue = asn1_values[member["default"]]["value"]
                    member_context["members"][0]["default_value"] = defaultValue
                    if asn1_values[member["default"]]["type"] == 'INTEGER':
                        member_context["members"][0]["default_type"] = simplestRosIntegerType(defaultValue, defaultValue)
            context["members"].extend(member_context["members"])

    # type aliases with multiple options
    elif type == "CHOICE":

        # add flag for indicating active option
        name = "choice"
        if "name" in asn1:
            name = f"{asn1['name']}_{name}"
        context["members"].append({
            "type": "uint8",
            "name": name,
        })

        # recursively add members for all options, incl. constant for flag
        for im, member in enumerate(asn1["members"]):
            if member is None:
                continue
            name = f"CHOICE_{camel2SNAKE(member['name'])}"
            member_context = asn1TypeToJinjaContext(t_name, member, asn1_types, asn1_values)
            member_context["members"][0]["constants"] = member_context["members"][0].get("constants", [])
            member_context["members"][0]["constants"].append({
                "type": "uint8",
                "name": name,
                "value": im
            })
            context["members"].extend(member_context["members"])

    # arrays
    elif type == "SEQUENCE OF":

        # add field for array
        array_name = asn1["name"] if "name" in asn1 else "array"
        array_type = asn1['element']['type']
        member_context = {
            "type": f"{array_type}[]",
            "type_snake": f"{camel2snake(array_type)}[]",
            "name": array_name,
            "constants": []
        }

        # add constants for size limits
        if "size" in asn1 and isinstance(asn1["size"][0], tuple):
            min_size = asn1["size"][0][0]
            max_size = asn1["size"][0][1]
            ros_type = simplestRosIntegerType(min_size, max_size)
            min_size_constant_name = "MIN_SIZE"
            max_size_constant_name = "MAX_SIZE"
            if "name" in asn1:
                min_constant_name = f"{camel2SNAKE(asn1['name'])}_{min_size_constant_name}"
                max_constant_name = f"{camel2SNAKE(asn1['name'])}_{max_size_constant_name}"
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(min_size_constant_name),
                "value": min_size
            })
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(max_size_constant_name),
                "value": max_size
            })

        context["members"].append(member_context)

    # enums
    elif type == "ENUMERATED":

        # choose simplest possible integer type
        values = [val[1] for val in asn1["values"] if val is not None]
        min_value = min(values)
        max_value = max(values)
        ros_type = simplestRosIntegerType(min_value, max_value)

        # add field for active value
        member_context = {
            "type": ros_type,
            "name": "value",
            "constants": [],
        }

        # add constants for all values
        for val in asn1["values"]:
            if val is None:
                continue
            member_context["constants"].append({
                "type": ros_type,
                "name": camel2SNAKE(val[0]),
                "value": val[1]
            })

        context["members"].append(member_context)

    # custom types
    elif type in asn1_types:

        name_cc = asn1["name"] if "name" in asn1 else "value"
        name = camel2snake(name_cc)
        context["members"].append({
            "type": validRosType(type),
            "name": validRosField(name),
            "name_cc": validRosField(name_cc)
        })

    elif type == "NULL":

        pass

    else:

        warnings.warn(f"Cannot handle type '{type}'")

    return context
