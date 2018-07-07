#!/usr/bin/env python3

import os
import subprocess

files = [f for f in os.listdir(".") if f.endswith(".svg")]

for f in files:
    subprocess.run([
        "inkscape", "-D", "-z", "--file=" + f,
        "--export-pdf=" + os.path.splitext(f)[0] + ".pdf"
    ])
