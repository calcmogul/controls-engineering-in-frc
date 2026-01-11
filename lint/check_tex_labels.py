#!/usr/bin/env python3

r"""
Verify all labels defined by \label commands are referenced at least once, and
verify that all ref commands refer to a defined label.
"""

import os
import re
import sys
from pathlib import Path


class Label:
    """
    Labels contains the LaTeX label's containing filename, line number on which
    it occurred, and its name.
    """

    def __init__(self, filename, line_number, name):
        self.filename = filename
        self.line_number = line_number
        self.name = name

    def __lt__(self, other):
        return self.filename < other.filename or (
            self.filename == other.filename and self.line_number < other.line_number
        )


def main():
    rgx = re.compile(r"\\(?P<command>(footref|eqref|ref|label)){(?P<arg>[^}]+)}")

    files = [
        f
        for f in Path(".").rglob("*")
        if f.suffix == ".tex" and not f.is_relative_to("./build/venv")
    ]

    labels = set()
    refs = set()

    # Maps from label name to tuple of filename and line number. This is used to
    # print error diagnostics.
    label_locations = {}
    ref_locations = {}

    for file in files:
        contents = file.read_text(encoding="utf-8")

        for match in rgx.finditer(contents):
            # Get line regex match was on
            linecount = 1
            for i in range(match.start()):
                if contents[i] == os.linesep:
                    linecount += 1

            if match.group("command") == "label":
                label = match.group("arg")
                labels.add(label)
                label_locations[label] = Label(file, linecount, label)
            elif "ref" in match.group("command"):
                ref = match.group("arg")
                refs.add(ref)
                ref_locations[ref] = Label(file, linecount, ref)

    undefined_refs = refs - labels
    unrefed_labels = labels - refs

    if labels == refs:
        # If labels and refs are equivalent sets, there are no undefined references
        # or unreferenced labels, so return success
        sys.exit(0)

    if undefined_refs:
        print(f"error: {len(undefined_refs)} undefined reference", end="")
        if len(undefined_refs) > 1:
            print("s", end="")
        print(":")

        # Print refs sorted by filename and line number
        for ref in sorted(ref_locations[l] for l in undefined_refs):
            print(f"[{ref.filename}:{ref.line_number}]\n    {ref.name}")

    if unrefed_labels:
        print(f"error: {len(unrefed_labels)} unreferenced label", end="")
        if len(unrefed_labels) > 1:
            print("s", end="")
        print(":")

        # Print labels sorted by filename and line number
        for label in sorted(label_locations[l] for l in unrefed_labels):
            print(f"[{label.filename}:{label.line_number}]\n    {label.name}")

    sys.exit(1)


if __name__ == "__main__":
    main()
