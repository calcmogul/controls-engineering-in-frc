#!/usr/bin/env python3

import multiprocessing as mp
import os
import subprocess
import sys


def run(name):
    subprocess.run([sys.executable, "-m", "black", "-q", name])


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("code")
    for f in fn
    if f.endswith(".py") and "current_limit.py" not in f
] + ["deputils.py", "format_code.py"]
files += [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("deps")
    for f in fn
    if f.endswith(".py")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
