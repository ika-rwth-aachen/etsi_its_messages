#!/usr/bin/env python

import argparse
import os
from typing import Dict, List

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


def findTypeDependencies(docs: Dict, type: str, log: bool = False, indent: int = 0) -> Dict[str, Dict]:

    relevant_type_infos = {}

    # find given type
    type_info = None
    for doc, info in docs.items():
        if type in info["types"].keys():
            type_info = info["types"][type]
            break
        if type in info["values"].keys():
            type_info = info["values"][type]
            break
    if type_info is None:
        return {}
    relevant_type_infos[type] = type_info
    if log:
        print(f"{' ' * indent}{type}")

    # get dependency types
    dependency_types = []
    if "members" in type_info:
        dependency_types += [m["type"] for m in type_info["members"] if m is not None and "SEQUENCE" not in m["type"]]
        dependency_types += [m["element"]["type"] for m in type_info["members"] if m is not None and "SEQUENCE" in m["type"]]
        dependency_types += [m["default"] for m in type_info["members"] if m is not None and "default" in m]
    if "element" in type_info:
        dependency_types += [type_info["element"]["type"]]
    
    # recursively find dependencies of dependencies
    for dependency_type in dependency_types:
        rti = findTypeDependencies(docs, dependency_type, log=log, indent=indent+2)
        relevant_type_infos.update(rti)
    
    return relevant_type_infos


def reduceAsn1Files(files: List[str], types: List[str], output_dir: str):

    for file in files:

        # read file and reduce
        with open(file, "r") as f:
            lines = f.readlines()
        reduced_lines = reduceAsn1File(lines, types)

        # write reduced file
        new_file = os.path.join(output_dir, os.path.basename(file))
        os.makedirs(output_dir, exist_ok=True)
        with open(new_file, "w") as f:
            for line in reduced_lines:
                f.write(line)


def reduceAsn1File(lines: List[str], types: List[str]) -> List[str]:

    reduced_lines = []
    copying = True # start copying header
    copying_type = False
    brace_level = 0

    # loop over all lines to check which ones to keep
    for line in lines:

        # always copy empty lines and keyword lines
        if line.isspace() or line.strip() in ("BEGIN", "END"):
            reduced_lines.append(line)
            continue

        # detect type to keep, entering copy mode
        if "::=" in line:
            if line.split("::=")[0].strip().split(" ")[0] in types:
                copying_type = True
                copying = False

        # copy line
        if copying_type or copying:
            reduced_lines.append(line)

        # leave copy mode when curly braces are closed
        brace_level += line.count("{")
        brace_level -= line.count("}")
        if brace_level == 0:
            copying_type = False

    return reduced_lines


def main():

    args = parseCli()

    docs = parseAsn1Files(args.files)

    relevant_type_infos = findTypeDependencies(docs, args.type, log=True)

    reduceAsn1Files(args.files, list(relevant_type_infos.keys()), args.output_dir)


if __name__ == "__main__":
    
    main()
