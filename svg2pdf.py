#!/usr/bin/env python3

import os
import subprocess

files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".svg")
]

for f in files:
    subprocess.run([
        "inkscape", "-D", "-z", "--file=" + f,
        "--export-pdf=" + os.path.splitext(f)[0] + ".pdf"
    ])
