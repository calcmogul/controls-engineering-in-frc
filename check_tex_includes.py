#!/usr/bin/env python3

"""Ensure every .tex file is transitively included by
controls-engineering-in-frc.tex
"""

import os
import re
import sys

ROOT = "controls-engineering-in-frc.tex"
files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".tex") and "build/venv/" not in dp
]

# Tuples are filename and whether filename has been parsed (whether node has
# been visited)
class Node:
    def __init__(self, filename):
        self.filename = filename
        self.visited = False


nodes = {f: Node(f) for f in files}
latex_vars = {}
error_occurred = False


def visit(filename):
    nodes[filename].visited = True

    # Get file contents
    with open(filename, "r") as f:
        contents = f.read()

    rgx = re.compile(
        r"""
        \\(?P<command>[a-z ]+)\*?      # Commaned name
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
                if not nodes[subfile].visited:
                    visit(subfile)
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


# Start at controls-engineering-in-frc.tex and perform depth-first search of
# file includes
visit(ROOT)

if not all(node.visited for node in nodes.values()):
    print(f"error: some .tex files were not transitively included in {ROOT}:")
    orphans = [node.filename for node in nodes.values() if not node.visited]
    for orphan in orphans:
        print("    " + orphan)
    sys.exit(1)
elif error_occurred:
    sys.exit(1)
else:
    sys.exit(0)
