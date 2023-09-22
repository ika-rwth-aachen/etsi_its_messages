#!/usr/bin/env python

import argparse
import os
from typing import Dict, List, Set

import asn1tools


def parseCli():

    parser = argparse.ArgumentParser(
        description="Reduces given ASN1 files to the type dependencies of a given type.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("files", type=str, nargs="+", help="ASN1 files")
    parser.add_argument("-t", "--type", type=str, required=True, help="ASN1 type")
    parser.add_argument("-o", "--output-dir", type=str, required=True, help="output directory")

    args = parser.parse_args()

    return args


def parseAsn1Files(files: List[str]) -> Dict:

    return asn1tools.parse_files(files)


def findTypeDependencies(type: str, docs: Dict, docs_to_search: List[str] = None, log: bool = False, indent: int = 0) -> Dict[str, Set[str]]:

    relevant_types_per_module = {doc: set() for doc in docs}

    for doc, doc_info in docs.items():
        for oset in doc_info["object-sets"]:
            relevant_types_per_module[doc].add(oset)
            # TODO
            # we need to find dependencies of the current object-set 'oset' --> parse the file because asn1tools does not support class?
            # Example: Reg-ConnectionManeuverAssist
            # Reg-ConnectionManeuverAssist	REG-EXT-ID-AND-TYPE ::= {
	        #   {ConnectionManeuverAssist-addGrpC  IDENTIFIED BY addGrpC},
	        #   ...
            # }
            # In the asn1tools-dict ConnectionManeuverAssist-addGrpC is not a member of Reg-ConnectionManeuverAssist

    # find given type
    type_info = None
    if docs_to_search is None:
        docs_to_search = list(docs.keys())
    for doc in docs_to_search:
        if doc not in docs:
            continue
        info = docs[doc]
        if type in info["types"].keys():
            type_info = info["types"][type]
            break
        if type in info["values"].keys():
            type_info = info["values"][type]
            break
        if type.split(".")[0] in info["object-classes"].keys():
            type=type.split(".")[0]
            type_info = info["object-classes"][type.split(".")[0]]
            break
    if type_info is None:
        return relevant_types_per_module
    module, module_info = doc, info
    relevant_types_per_module[module].add(type)
    if log:
        print(f"{' ' * indent}{type}")

    # get dependency types
    dependency_types = []
    if "members" in type_info:
        for m in type_info["members"]:
            if m is None:
                continue
            if m["type"] in ("SEQUENCE", "SEQUENCE OF"):
                dependency_types.append(m["element"]["type"])
            elif m["type"] == "CHOICE":
                dependency_types += [mm["type"] for mm in m["members"] if mm is not None]
            else:
                dependency_types.append(m["type"])
            if "default" in m:
                dependency_types.append(m["default"])
    if "element" in type_info:
        dependency_types += [type_info["element"]["type"]]

    # recursively find dependencies of dependencies
    for dependency_type in dependency_types:
        dependency_docs = [module]
        for doc, imports_from_doc in module_info["imports"].items():
            if dependency_type in imports_from_doc:
                dependency_docs = [doc]
                break
        rt = findTypeDependencies(dependency_type, docs, dependency_docs, log=log, indent=indent+2)
        for doc in docs:
            if doc in rt:
                relevant_types_per_module[doc] = relevant_types_per_module[doc].union(rt[doc])

    return relevant_types_per_module


def reduceAsn1Files(files: List[str], relevant_types_per_module: Dict[str, Set[str]], output_dir: str):

    for file in files:

        # read file and reduce
        with open(file, "r") as f:
            lines = f.readlines()
        reduced_lines = reduceAsn1File(lines, relevant_types_per_module)

        # write reduced file
        new_file = os.path.join(output_dir, os.path.basename(file))
        os.makedirs(output_dir, exist_ok=True)
        with open(new_file, "w") as f:
            for line in reduced_lines:
                f.write(line)


def reduceAsn1File(lines: List[str], relevant_types_per_module: Dict[str, Set[str]]) -> List[str]:

    reduced_lines = []

    current_module = None
    brace_level = 0
    is_between_begin_end = False
    modules_with_definitions = set()

    # find modules that have relevant definitions
    for line in lines:

        # detect when inside module
        if "BEGIN" in line:
            is_between_begin_end = True
        if "END" in line:
            is_between_begin_end = False

        # detect module name
        if brace_level == 0 and not is_between_begin_end and "{" in line and " " not in line.split("{")[0].strip():
            current_module = line.split("{")[0].strip()

        # detect relevant type to keep, saving current module name
        if current_module not in relevant_types_per_module:
            continue
        if "::=" in line:
            if line.split("::=")[0].strip().split(" ")[0] in relevant_types_per_module[current_module]:
                modules_with_definitions.add(current_module)

        brace_level += line.count("{")
        brace_level -= line.count("}")

    current_module = None
    brace_level = 0
    is_between_begin_end = False
    copying = True
    copying_type = False

    # second pass over all lines to decide which ones to keep
    for line in lines:

        # detect when inside module
        if "BEGIN" in line:
            is_between_begin_end = True
        if "END" in line:
            is_between_begin_end = False

        # detect module name
        if brace_level == 0 and not is_between_begin_end and "{" in line and " " not in line.split("{")[0].strip():
            current_module = line.split("{")[0].strip()

        # skip modules without relevant definitions
        if current_module is not None and current_module not in modules_with_definitions:
            continue

        # always copy empty lines, comments, and keyword lines
        if line.isspace() or line.startswith("--") or "BEGIN" in line or "END" in line:
            reduced_lines.append(line)
            continue

        # start copying until first definition
        if not is_between_begin_end:
            copying = True
        elif is_between_begin_end and "::=" in line:
            copying = False

        # detect relevant type to keep, entering copy mode
        if current_module not in relevant_types_per_module:
            continue
        if "::=" in line:
            if line.split("::=")[0].strip().split()[0] in relevant_types_per_module[current_module]:
                copying_type = True
                modules_with_definitions.add(current_module)

        # copy line
        if copying_type or copying:
            reduced_lines.append(line)

        # leave copy mode when curly braces are closed
        brace_level += line.count("{")
        brace_level -= line.count("}")
        if brace_level == 0:
            copying_type = False

    # remove consecutive blank lines
    prev_line_is_space = False
    reduced_lines_less_blanks = []
    for line in reduced_lines:
        if not line.isspace() or not prev_line_is_space:
            reduced_lines_less_blanks.append(line)
        prev_line_is_space = line.isspace()

    return reduced_lines_less_blanks


def main():

    args = parseCli()

    docs = parseAsn1Files(args.files)

    relevant_types_per_module = findTypeDependencies(args.type, docs, log=True)

    reduceAsn1Files(args.files, relevant_types_per_module, args.output_dir)


if __name__ == "__main__":

    main()
