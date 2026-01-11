#!/usr/bin/env python3

"""
Ensure every .tex file is transitively included by
controls-engineering-in-frc.tex
"""

import os
import re
import sys
from pathlib import Path


class Node:
    """
    Tuples are filename and whether filename has been parsed (whether node has
    been visited)
    """

    def __init__(self, filename: Path):
        self.filename = filename
        self.visited = False


def visit(filename: Path, nodes: dict[Path, Node], latex_vars: dict[str, str]) -> bool:
    """
    Recurse through a file's includes.

    Returns:
        True if error occurred.
    """

    error_occurred: bool = False
    nodes[filename].visited = True

    # Ignore files that break parsing
    if filename.is_relative_to("./preamble"):
        return False

    # Get file contents
    with open(filename, "r", encoding="utf-8") as f:
        contents = f.read()

    rgx = re.compile(
        r"""
        \\(?P<command>[a-z ]+)\*?      # Command name
        ({(?P<arg1>[^}]+)}             # First argument
         (?P<arg_count>\[[0-9]+\])?)?  # Optional square bracket arg count
        ({(?P<arg2>[^}]+)})?""",
        re.VERBOSE | re.MULTILINE,
    )

    for match in rgx.finditer(contents):
        if match.group("command") == "renewcommand":
            if match.group("arg2"):
                latex_vars[match.group("arg1")] = match.group("arg2")
        elif match.group("command") in ["include", "input"]:
            # Files are included via \include{file.tex} or \input{file.tex}
            subfile = match.group("arg1") + ".tex"

            # Recursively substitute any variables until no changes occur
            old_subfile = ""
            while old_subfile != subfile:
                old_subfile = subfile
                for var in latex_vars.items():
                    subfile = subfile.replace(var[0], var[1])

            try:
                if not nodes[Path(subfile)].visited:
                    error_occurred |= visit(Path(subfile), nodes, latex_vars)
            except KeyError:
                # Get line regex match was on
                linecount = 1
                for i in range(match.start()):
                    if contents[i] == os.linesep:
                        linecount += 1
                print(
                    f"[{filename}:{linecount}] error: included file '{subfile}' does not exist"
                )
                error_occurred = True
    return error_occurred


def check(root: Path, other_root_names: list[str]):
    files: list[Path] = [
        f
        for f in Path(".").rglob("*")
        if f.suffix == ".tex"
        and not f.is_relative_to("./build/venv")
        and f.name not in other_root_names
    ]
    nodes: dict[Path, Node] = {f: Node(f) for f in files}
    latex_vars: dict[str, str] = {}

    # Start at root .tex file and perform depth-first search of file includes
    error_occurred: bool = visit(root, nodes, latex_vars)

    if not all(node.visited for node in nodes.values()):
        orphans = [node.filename for node in nodes.values() if not node.visited]

        print(f"error: {len(orphans)} .tex file", end="")
        if len(orphans) > 1:
            print("s", end="")
        print(f" not transitively included in {root}:")

        for orphan in orphans:
            print("    " + orphan.as_posix())
        sys.exit(1)
    elif error_occurred:
        sys.exit(1)


def main():
    check(
        Path("controls-engineering-in-frc-ebook.tex"),
        ["controls-engineering-in-frc-printer.tex"],
    )
    check(
        Path("controls-engineering-in-frc-printer.tex"),
        ["controls-engineering-in-frc-ebook.tex"],
    )


if __name__ == "__main__":
    main()
