#!/usr/bin/env python

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

import re
import warnings
from typing import Dict, List, Optional, Tuple

import asn1tools
import numpy as np


ASN1_PRIMITIVES_2_ROS = {
    "BOOLEAN": "bool",
    "INTEGER": "int64",
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

    # exampleString-WithNumber123 -> example_string-_With_Number123
    ss = re.sub("([A-Z])", r"_\1", s)

    # example_string-_With_Number123 -> EXAMPLE_STRING_WITH_NUMBER123
    ss = ss.upper().lstrip("_").replace("-", "_").replace("__", "_")

    # EXAMPLE_STRING_WITH_NUMBER123 -> EXAMPLE_STRING_WITH_NUMBER_123
    ss = re.sub("([A-Z])([0-9])", r"\1_\2", ss)

    # special cases
    ss = ss.replace("C_A_M", "CAM")
    ss = ss.replace("D_E_N_M", "DENM")
    ss = ss.replace("S_P_A_T_E_M", "SPATEM")
    ss = ss.replace("S_P_A_T", "SPAT")
    ss = ss.replace("D_S_R_C", "DSRC")
    ss = ss.replace("R_S_U", "RSU")
    ss = ss.replace("W_M_I", "WMI")
    ss = ss.replace("V_DS", "VDS")
    ss = ss.replace("_I_D", "_ID")
    ss = ss.replace("_U_T_C", "_UTC")
    ss = ss.replace("G_N_S_S", "GNSS")
    ss = ss.replace("GNSSPLUS", "GNSS_PLUS")

    return ss

def camel2snake(s: str) -> str:
    """Converts a camelCase string to snake_case.

    Args:
        s (str): camelCaseString

    Returns:
        str: snake_case_string
    """

    # exampleString-WithNumber123 -> example_string-_With_Number123
    ss = re.sub("([A-Z])", r"_\1", s)

    # example_string-_With_Number123 -> example_string_with_number123
    ss = ss.lower().lstrip("_").replace("-", "_").replace("__", "_")

    # example_string_with_number123 -> example_string_with_number_123
    ss = re.sub("([a-z])([0-9])", r"\1_\2", ss)

    # special cases
    ss = ss.replace("c_a_m", "cam")
    ss = ss.replace("d_e_n_m", "denm")
    ss = ss.replace("s_p_a_t_e_m", "spatem")
    ss = ss.replace("s_p_a_t", "spat")
    ss = ss.replace("m_a_p_e_m", "mapem")
    ss = ss.replace("m_a_p", "map")
    ss = ss.replace("d_s_r_c", "dsrc")
    ss = ss.replace("r_s_u", "rsu")
    ss = ss.replace("w_m_i", "wmi")
    ss = ss.replace("v_d_s", "vds")
    ss = ss.replace("_i_d", "_id")
    ss = ss.replace("_u_t_c", "_utc")
    ss = ss.replace("wmin", "wm_in")
    ss = ss.replace("_3_d", "3_d")
    ss = ss.replace("_b_1", "_b1")
    ss = ss.replace("_x_y_2", "_xy2")
    ss = ss.replace("_x_y_3", "_xy3")
    ss = ss.replace("_x_y", "_xy")
    ss = ss.replace("_l_lm_d_6", "_l_lm_d6")

    return ss

def validRosType(s: str) -> str:
    """Converts a string to make it a valid ROS message type.

    Args:
        s (str): Not-A-Message-Type

    Returns:
        str: A_Message_Type
    """

    return s.replace("-", "")


def validRosField(s: str, is_const: bool = False) -> str:
    """Converts a string to make it a valid ROS message field name.

    Args:
        s (str): Not-A-Ros-Message-Field
        is_const (bool): whether the field is a constant

    Returns:
        str: a_ros_message_field or A_ROS_MESSAGE_CONSTANT
    """

    ss = s.upper() if is_const else s.lower()

    # avoid C/C++ keywords
    if ss == "long":
        ss = "lon"
    if ss == "class":
        ss = "cls"

    return ss


def validCField(s: str, is_const: bool = False) -> str:
    """Converts a string to make it a valid C message field name.

    Args:
        s (str): e.g., "class"

    Returns:
        str: e.g., "Class"
    """

    # avoid C/C++ keywords
    ss = s.replace("-", "_")
    if ss == "long":
        ss = "Long"
    if ss == "class":
        ss = "Class"

    return ss


def noSpace(s: str) -> str:
    """Replaces any spaces in a string with underscores.

    Args:
        s (str): my string

    Returns:
        str: my_string
    """

    return s.replace(" ", "_")


def noDash(s: str) -> str:
    """Replaces any dashes in a string with nothing.

    Args:
        s (str): my string

    Returns:
        str: my_string
    """

    return s.replace("-", "")


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
                if line.rstrip().endswith("{"):
                    type = line.split("::=")[0].split("{")[0].strip().split()[0]
                    raw_def = ""
                elif len(line.split("::=")) == 2:
                    type = line.split("::=")[0].strip().split()[0]
                    if "}" in line or not ("{" in line or "}" in line):
                        raw_def = line
                        asn1_raw[type] = raw_def
                        raw_def = None
                    else:
                        raw_def = ""
            if raw_def is not None:
                raw_def += line
                if "}" in line and not "}," in line and not ("::=" in line and line.rstrip().endswith("{")):
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
    """Extracts all parsed ASN1 value information from multiple ASN1 documents.

    Args:
        asn1_docs (Dict): type information by document

    Raises:
        ValueError: if a type is found in multiple documents

    Returns:
        Dict[str, Dict]: value information by name
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
        asn1_values (Dict[str, Dict]): value information of all values by name

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
        "t_name_camel": noDash(t_name),
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
            "name": validRosField(camel2snake(name)),
            "name_cc": validCField(name),
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
                min_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{min_constant_name}", is_const=True)
                max_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{max_constant_name}", is_const=True)
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(min_constant_name, is_const=True),
                "value": min_value
            })
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(max_constant_name, is_const=True),
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
                    min_size_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{min_size_constant_name}", is_const=True)
                    max_size_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{max_size_constant_name}", is_const=True)
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(min_size_constant_name, is_const=True),
                    "value": min_size
                })
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(max_size_constant_name, is_const=True),
                    "value": max_size
                })
            else:
                size = asn1["size"][0]
                ros_type = simplestRosIntegerType(size, size)
                size_constant_name = "SIZE" if type != "BIT STRING" else "SIZE_BITS"
                if "name" in asn1:
                    size_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{size_constant_name}", is_const=True)
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(size_constant_name, is_const=True),
                    "value": size
                })

        # add constants for named numbers
        if "named-numbers" in asn1:
            for k, v in asn1["named-numbers"].items():
                constant_name = validRosField(camel2SNAKE(k))
                if "name" in asn1:
                    constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{constant_name}", is_const=True)
                member_context["constants"].append({
                    "type": ros_type,
                    "name": validRosField(constant_name, is_const=True),
                    "value": v
                })

        # add index constants for named bits
        if "named-bits" in asn1:
            for k, v in asn1["named-bits"]:
                constant_name = camel2SNAKE(k)
                member_context["constants"].append({
                    "type": "uint8",
                    "name": validRosField(f"BIT_INDEX_{constant_name}", is_const=True),
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
                if member["default"] in asn1_values:
                    asn1_value = asn1_values[member["default"]]
                    default_value = asn1_value["value"]
                    default_name = validRosField(f"DEFAULT_{camel2SNAKE(member['name'])}", is_const=True)
                    if asn1_value["type"] == 'INTEGER':
                        default_type = simplestRosIntegerType(default_value, default_value)
                    else:
                        default_type = ASN1_PRIMITIVES_2_ROS[asn1_value["type"]]
                    member_context["members"][0]["default"] = {
                        "type": default_type,
                        "name": default_name,
                        "value": default_value
                    }
            context["members"].extend(member_context["members"])

    # type aliases with multiple options
    elif type == "CHOICE":

        # add flag for indicating active option
        name = "choice"
        if "name" in asn1:
            name = f"{asn1['name']}_{name}"
        name = validRosField(camel2snake(name))
        context["members"].append({
            "type": "uint8",
            "name": name,
            "is_choice_var": True
        })

        # recursively add members for all options, incl. constant for flag
        for im, member in enumerate(asn1["members"]):
            if member is None:
                continue
            member_name = validRosField(f"CHOICE_{camel2SNAKE(member['name'])}", is_const=True)
            if "name" in asn1:
                member_name = validRosField(f"CHOICE_{camel2SNAKE(asn1['name'])}_{camel2SNAKE(member['name'])}", is_const=True)
            member_context = asn1TypeToJinjaContext(t_name, member, asn1_types, asn1_values)
            if len(member_context["members"]) > 0:
                if "name" in asn1:
                    member_context["members"][0]["choice_name"] = validCField(asn1["name"])
                    member_context["members"][0]["choice_option_name"] = validCField(member_context["members"][0]["name_cc"])
                    member_context["members"][0]["name"] = validRosField(f"{camel2snake(asn1['name'])}_{camel2snake(member_context['members'][0]['name'])}")
                    member_context["members"][0]["name_cc"] = validCField(f"{asn1['name']}_{member_context['members'][0]['name_cc']}")
                member_context["members"][0]["is_choice"] = True
                member_context["members"][0]["choice_var_name"] = name
                member_context["members"][0]["constants"] = member_context["members"][0].get("constants", [])
                for c_idx, constant in enumerate(member_context["members"][0]["constants"]):
                    member_context["members"][0]["constants"][c_idx]["name"] = validRosField(f"{member_context['members'][0]['name']}_{constant['name']}", is_const=True)
                member_context["members"][0]["constants"].append({
                    "type": "uint8",
                    "name": member_name,
                    "value": im
                })
            context["members"].extend(member_context["members"])

    # arrays
    elif type == "SEQUENCE OF":

        # add field for array
        array_name = asn1["name"] if "name" in asn1 else "array"
        array_type = asn1['element']['type']
        member_context = {
            "t_name": array_type,
            "type": f"{array_type}[]",
            "type_snake": f"{camel2snake(array_type)}[]",
            "name": validRosField(camel2snake(array_name)),
            "name_cc": validCField(array_name),
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
                min_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{min_size_constant_name}", is_const=True)
                max_constant_name = validRosField(f"{camel2SNAKE(asn1['name'])}_{max_size_constant_name}", is_const=True)
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(min_size_constant_name, is_const=True),
                "value": min_size
            })
            member_context["constants"].append({
                "type": ros_type,
                "name": validRosField(max_size_constant_name, is_const=True),
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
                "name": validRosField(camel2SNAKE(val[0]), is_const=True),
                "value": val[1]
            })

        context["members"].append(member_context)

    # custom types
    elif type in asn1_types:

        name_cc = asn1["name"] if "name" in asn1 else "value"
        name = camel2snake(name_cc)
        context["members"].append({
            "t_name": type,
            "type": validRosType(type),
            "name": validRosField(camel2snake(name)),
            "name_cc": validCField(name_cc)
        })

    elif type == "NULL":

        pass

    else:

        warnings.warn(f"Cannot handle type '{type}'")

    return context
