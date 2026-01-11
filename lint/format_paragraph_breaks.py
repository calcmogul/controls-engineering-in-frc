#!/usr/bin/env python3

r"""
Removes blank lines before \begin blocks since those add an undesired paragraph
break. Also ensure a blank line exists between \end statements and new
paragraphs.
"""

import re
from pathlib import Path


def main():
    begin_rgx = re.compile(r"\n [ ]* (\n [ ]* \\begin\{ [^\}]+ \})", re.VERBOSE)
    end_rgx = re.compile(r"(\\end \{ [^\}]+ \}) (?=\n[\w\$])", re.VERBOSE)

    files: list[Path] = [
        f
        for f in Path(".").rglob("*")
        if f.suffix == ".tex" and not f.is_relative_to("./build/venv")
    ]

    for f in files:
        in_contents = f.read_text(encoding="utf-8")

        out_contents = begin_rgx.sub(r"\g<1>", in_contents)
        out_contents = end_rgx.sub(r"\g<1>\n", out_contents)
        out_contents = re.sub(r"\n\n\n", r"\n\n", out_contents)

        if in_contents != out_contents:
            f.write_text(out_contents, encoding="utf-8")


if __name__ == "__main__":
    main()
