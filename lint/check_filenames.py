#!/usr/bin/env python3

"""
Ensure .tex filenames use hyphens and .py files use underscores. Vice versa is
not allowed.
"""

import os
import sys

files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if (f.endswith(".tex") or f.endswith(".bib")) and "build/" not in dp
]

error_occurred = False
for filename in files:
    filename = os.path.basename(filename)
    if "_" in filename:
        print(f"error: filename '{filename}' should not include underscores")
        error_occurred = True

files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".py") and "build/" not in dp
]

error_occurred = False
for filename in files:
    filename = os.path.basename(filename)
    if "-" in filename:
        print(f"error: filename '{filename}' should not include hyphens")
        error_occurred = True

if error_occurred:
    sys.exit(1)
else:
    sys.exit(0)
