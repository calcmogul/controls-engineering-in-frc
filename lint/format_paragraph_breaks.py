#!/usr/bin/env python3

r"""
Removes blank lines before \begin blocks since those add an undesired paragraph
break. Also ensure a blank line exists between \end statements and new
paragraphs.
"""

import os
import re

begin_rgx = re.compile(r"\n [ ]* (\n [ ]* \\begin\{ [^\}]+ \})", re.VERBOSE)
end_rgx = re.compile(r"(\\end \{ [^\}]+ \}) (?=\n[\w\$])", re.VERBOSE)

files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".tex") and "build/venv/" not in dp
]

for filename in files:
    with open(filename, "r", encoding="utf-8") as infile:
        in_contents = infile.read()

    out_contents = begin_rgx.sub(r"\g<1>", in_contents)
    out_contents = end_rgx.sub(r"\g<1>\n", out_contents)
    out_contents = re.sub(r"\n\n\n", r"\n\n", out_contents)

    if in_contents != out_contents:
        with open(filename + ".tmp", "w", encoding="utf-8") as outfile:
            outfile.write(out_contents)
            os.remove(filename)
            os.rename(filename + ".tmp", filename)
