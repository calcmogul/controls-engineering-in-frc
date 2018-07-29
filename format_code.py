#!/usr/bin/env python3

import os
import subprocess
import sys

files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("code")
    for f in fn
    if f.endswith(".py")
]

subprocess.run(
    [sys.executable, "-m", "yapf", "--style", "google", "-i", "format_code.py"])
for f in files:
    subprocess.run([sys.executable, "-m", "yapf", "--style", "google", "-i", f])
