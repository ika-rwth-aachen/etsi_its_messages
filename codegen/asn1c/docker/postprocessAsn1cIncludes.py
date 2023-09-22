#!/usr/bin/env python3

import argparse
import os
import re
import shutil


def parseCli():

    parser = argparse.ArgumentParser(
        description="Postprocesses asn1c-generated header and source files to introduce an extra layer in the include folder.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("parent_path", type=str, help="parent folder of asn1c-generated header (include/) and source files (src/)")

    args = parser.parse_args()

    return args


def adjustIncludes(parent_path: str):

    prefix = os.path.basename(parent_path)

    header_files = [os.path.join(parent_path, "include", f) for f in os.listdir(os.path.join(parent_path, "include"))]
    source_files = [os.path.join(parent_path, "src", f) for f in os.listdir(os.path.join(parent_path, "src"))]
    header_files = [f for f in header_files if os.path.isfile(f)]
    source_files = [f for f in source_files if os.path.isfile(f)]
    headers = [os.path.basename(f) for f in header_files]

    for file in [*header_files, *source_files]:
        print(file)
        with open(file, "r") as f:
            contents = f.read()
            for header in headers:
                if re.search(r'^#include\s+"{}"'.format(header), contents, re.MULTILINE):
                    contents = re.sub(r'(^#include\s+")({}")'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
                if re.search(r'^#include\s+<{}>'.format(header), contents, re.MULTILINE):
                    contents = re.sub(r'(^#include\s+<)({}>)'.format(re.escape(header)), r'\1{}/\2'.format(prefix), contents, flags=re.MULTILINE)
        with open(file, "w") as f:
            f.write(contents)


def moveHeaders(parent_path: str):

    include_path = os.path.join(parent_path, "include")
    new_include_path = os.path.join(parent_path, "include", os.path.basename(parent_path))
    os.makedirs(new_include_path, exist_ok=True)
    header_files = [os.path.join(include_path, f) for f in os.listdir(include_path)]
    header_files = [f for f in header_files if os.path.isfile(f)]
    for f in header_files:
        shutil.move(f, os.path.join(new_include_path, os.path.basename(f)))


def main():

    args = parseCli()
    args.parent_path = os.path.realpath(args.parent_path)
    adjustIncludes(args.parent_path)
    moveHeaders(args.parent_path)


if __name__ == "__main__":
    
    main()
