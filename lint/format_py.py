#!/usr/bin/env python3

import multiprocessing as mp
import os
import re
import subprocess
import sys


def run_py(name):
    subprocess.run([sys.executable, "-m", "black", "-q", name])


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk(".")
    for f in fn
    if f.endswith(".py") and not re.search("^\./build|\.egg-info/", dp)
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run_py, files)
