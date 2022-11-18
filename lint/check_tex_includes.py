#!/usr/bin/env python3

"""
Ensure every .tex file is transitively included by
controls-engineering-in-frc.tex
"""

import os
import re
import sys

EBOOK_ROOT = "controls-engineering-in-frc-ebook.tex"
PRINTER_ROOT = "controls-engineering-in-frc-printer.tex"


class Node:
    """
    Tuples are filename and whether filename has been parsed (whether node has
    been visited)
    """

    def __init__(self, filename):
        self.filename = filename
        self.visited = False


# Configure visit()'s state for ebook files
ebook_files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".tex")
    and "build/venv/" not in dp
    and f != "controls-engineering-in-frc-printer.tex"
]
nodes = {f: Node(f) for f in ebook_files}
latex_vars = {}
error_occurred = False


def visit(filename):
    """Recurse through a file's includes."""
    nodes[filename].visited = True

    # Ignore files that break parsing
    if "preamble/" in filename:
        return

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
                # pragma pylint: disable=global-statement
                global error_occurred
                error_occurred = True


# Start at root .tex file and perform depth-first search of file includes
visit(EBOOK_ROOT)

if not all(node.visited for node in nodes.values()):
    orphans = [node.filename for node in nodes.values() if not node.visited]

    print(f"error: {len(orphans)} .tex file", end="")
    if len(orphans) > 1:
        print("s", end="")
    print(f" not transitively included in {EBOOK_ROOT}:")

    for orphan in orphans:
        print("    " + orphan)
    sys.exit(1)
elif error_occurred:
    sys.exit(1)
else:
    sys.exit(0)

# Configure visit()'s state for printer files
printer_files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".tex")
    and "build/venv/" not in dp
    and f != "controls-engineering-in-frc-ebook.tex"
]
nodes = {f: Node(f) for f in printer_files}
latex_vars = {}
error_occurred = False

# Start at root .tex file and perform depth-first search of file includes
visit(PRINTER_ROOT)

if not all(node.visited for node in nodes.values()):
    orphans = [node.filename for node in nodes.values() if not node.visited]

    print(f"error: {len(orphans)} .tex file", end="")
    if len(orphans) > 1:
        print("s", end="")
    print(f" not transitively included in {EBOOK_ROOT}:")

    for orphan in orphans:
        print("    " + orphan)
    sys.exit(1)
elif error_occurred:
    sys.exit(1)
else:
    sys.exit(0)
