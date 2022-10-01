#!/usr/bin/env python3

"""Removes trailing whitespace from .tex files."""

import os


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".tex") and not dp.startswith("./build")
]

for file in files:
    with open(file, encoding="utf-8") as f:
        lines = f.read()

    output = "".join(line.rstrip() + "\n" for line in lines.splitlines())
    if lines != output:
        with open(file, "w", encoding="utf-8") as f:
            f.write(output)
