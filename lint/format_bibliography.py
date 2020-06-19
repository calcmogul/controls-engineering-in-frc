#!/usr/bin/env python3

"""Formats and sorts bibliography entries in lexographic order."""

import re
import sys


class Entry:
    def __init__(self, type, label, fields):
        """Construct an Entry

        Keyword arguments:
        type -- bibliography entry type (e.g., misc, online)
        label -- LaTeX label
        fields -- dictionary of field types and values
        """
        self.type = type
        self.label = label
        self.fields = fields

    def __lt__(self, other):
        return self.label < other.label


# Example:
# @misc{bib:calculus_taylor_series,
#   author = {3Blue1Brown},
#   title = {Taylor series},
#   url = {https://www.youtube.com/watch?v=3d6DsjIBzJ4},
#   year = {2017}
# }
entry_regex = re.compile(
    r"""@(?P<type>[a-z]+) \{ (?P<label>[\w:-]+) \s*
(?P<fields>(, \s* \w+ \s* = \s* \{ [^\}]+ \} \s*)+)
\}""",
    re.VERBOSE,
)
keyval_regex = re.compile(r"(?P<key>\w+) \s* = \s* \{ (?P<value>[^\}]+) \}", re.VERBOSE)

with open("controls-engineering-in-frc.bib") as f:
    contents = f.read()

entries = []
for match in entry_regex.finditer(contents):
    fields = {}

    for field in keyval_regex.finditer(match.group("fields")):
        fields[field.group("key")] = field.group("value")

    entries.append(Entry(match.group("type"), match.group("label"), fields))
entries.sort()

error_occurred = False

# Verify labels start with "bib:"
for entry in entries:
    if not entry.label.startswith("bib:"):
        print(f"error: label '{entry.label}' should be prefixed with 'bib:'")
        error_occurred = True
if error_occurred:
    sys.exit(1)

# Write formatted bibliography entries back out
output = ""
for i, entry in enumerate(entries):
    if i != 0:
        output += "\n"

    output += f"@{entry.type}"
    output += "{"
    output += f"{entry.label}"
    output += ",\n"
    keys = sorted(entry.fields.keys())
    for j, key in enumerate(keys):
        output += f"  {key} = "
        output += "{"
        output += f"{entry.fields[key]}"
        output += "}"
        if j < len(keys) - 1:
            output += ",\n"
        else:
            output += "\n}\n"

with open("controls-engineering-in-frc.bib", "w") as f:
    f.write(output)
