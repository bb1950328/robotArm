import os
import re
from typing import List

import file_hasher

END_TAG = "//END_CPP_LIB"
START_TAG = "//START_CPP_LIB"

obsolete_lines = [
    "#include <cmath>",
    "#include <iostream>",
    '#include "iostream"',
    "#include <ctime>",
    r'#include "../h/.*\.hpp"',
    r'//\s*',
    r'// Created by .* on .*',
    r"#define PI .*",
]

file_blacklist = [
    "main.cpp",
    "util.cpp",
    "util.hpp",
]

replaces = [
    ["util::radians", "radians"],
    ["util::degrees", "degrees"],
    ["std::sin", "sin"],
    ["std::cos", "cos"],
    ["std::tan", "tan"],
    ["std::asin", "asin"],
    ["std::acos", "acos"],
    ["std::atan", "atan"],
    ["std::sqrt", "sqrt"],
    ["std::ceil", "ceil"],
]

EOL = "\n"

ino_file = r"D:\SchuleDev\robotArm\ArduinoRobotArm\ArduinoRobotArm.ino"
cpp_base_dir = os.path.abspath(os.path.join(".", "..", "pc_cpp"))


def convert_cout(cout_line: str) -> List[str]:
    parts = [n.strip() for n in cout_line.split("<<")]
    if parts[-1].endswith(";"):
        parts[-1] = parts[-1].strip(";")
    if parts[0] != "cout":
        raise ValueError(f"Invalid cout line: {cout_line}")
    result = []
    for p in parts[1:]:
        if p == "EOL":
            result[-1] = result[-1].replace("print", "println", 1)
            result.append("")
        else:
            result.append(f"Serial.print({p});")
    return result


def read_file(path: str, include_comment=True) -> List[str]:
    with open(path) as fi:
        comment = f"//Start of {os.path.basename(path)}".ljust(80, "*")
        file_lines = [line.strip() for line in fi.readlines()]
        if include_comment:
            return [comment] + file_lines + [comment.replace("Start", "End") + "*"]
        else:
            return file_lines


def clean_file(inp: List[str]) -> List[str]:
    result = []
    for line in inp:
        if not is_obsolete_line(line):
            if line.strip().startswith("cout"):
                result.extend(convert_cout(line))
            else:
                result.append(line)
    return result


def is_obsolete_line(line) -> bool:
    for pattern in obsolete_lines:
        if re.fullmatch(pattern, line):
            return True
    return False


def dependency_sort(li, deps) -> list:
    graph = {n: [] for n in li}
    for a, b in deps:
        graph[a].append(b)
    result = []

    while graph:
        keys = list(graph.keys())
        for k in keys:
            if len(graph[k]) == 0 or set(graph[k]).issubset(set(result)):
                graph.pop(k)
                result.append(k)
                break

    return result


def clean_emty_lines(lines: List[str]) -> List[str]:
    output = []
    last_was_empty = False
    for li in lines:
        this_is_empty = li.strip() == ""
        if this_is_empty:
            li = ""
        if last_was_empty:
            if not this_is_empty:
                output.append(li)
        else:
            output.append(li)
    return output


if __name__ == '__main__':
    before_md5 = file_hasher.get_md5(ino_file)

    hpp_dir = os.path.join(cpp_base_dir, "h")
    cpp_dir = os.path.join(cpp_base_dir, "src")
    hpp_paths = [os.path.abspath(os.path.join(hpp_dir, fi)) for fi in os.listdir(hpp_dir) if fi not in file_blacklist]
    cpp_paths = [os.path.abspath(os.path.join(cpp_dir, fi)) for fi in os.listdir(cpp_dir) if fi not in file_blacklist]

    hpp_files = {os.path.basename(path): read_file(path) for path in hpp_paths}
    cpp_files = {os.path.basename(path): read_file(path) for path in cpp_paths}

    depending_files = []  # [a, b] # a depends on b

    for fname, fi in hpp_files.items():
        i = 0
        while i < len(fi):
            if re.fullmatch(r'#include "\w+\.hpp"', fi[i]):
                depending_files.append([os.path.join(hpp_dir, fname),
                                        os.path.join(hpp_dir, fi[i].split('"')[1])])
                del fi[i]
            else:
                i += 1

    hpp_paths = dependency_sort(hpp_paths, depending_files)

    for a, b in depending_files:
        print(f"{os.path.basename(a)} depends on {os.path.basename(b)}")

    with open(ino_file) as f:
        everything = f.read()
        if START_TAG not in everything:
            raise ValueError(START_TAG + "not in file!")
        if END_TAG not in everything:
            raise ValueError(END_TAG + "not in file!")
        before_lib = everything.split(START_TAG)[0]
        after_lib = everything.split(END_TAG)[-1]
    lib = []

    for fpath in hpp_paths:
        fname = os.path.basename(fpath)
        cleaned = clean_file(hpp_files[fname])
        lib.extend(cleaned)
    for fname in cpp_files.keys():
        cleaned = clean_file(cpp_files[fname])
        lib.extend(cleaned)

    for rep in replaces:
        lib = [line.replace(*rep) for line in lib]

    indent = 0
    for i in range(len(lib)):
        indent -= lib[i].count("}")
        lib[i] = indent * 2 * " " + lib[i]
        indent += lib[i].count("{")

    lib = clean_emty_lines(lib)
    with open(ino_file, "w") as f:
        f.write(before_lib)
        f.write(START_TAG + EOL)
        for line in lib:
            f.write(line + EOL)
        f.write(END_TAG)
        f.write(after_lib)

    after_md5 = file_hasher.get_md5(ino_file)

    print(f"INFO: copied {len(lib)} lines. {len(list(filter(lambda li: li.strip(), lib)))} of them aren't empty.")
    if before_md5 == after_md5:
        print(f"WARNING: file hash has not changed: {before_md5}")
    else:
        print(f"INFO: file hash has changed from {before_md5} to {after_md5}")
