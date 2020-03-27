#!/usr/bin/env python3

import json
import multiprocessing as mp
import os
import subprocess
import sys


def run_py(name):
    subprocess.run([sys.executable, "-m", "black", "-q", name])


def run_json(name):
    with open(name) as file:
        input = file.read()
    output = json.dumps(json.loads(input), sort_keys=True, indent=4) + "\n"
    if input != output:
        with open(name, "w") as file:
            file.write(output)


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("code")
    for f in fn
    if f.endswith(".py") and "current_limit.py" not in f
] + ["lint/check_links.py", "lint/check_tex_includes.py", "lint/format_code.py"]
files += [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("deps")
    for f in fn
    if f.endswith(".py")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run_py, files)

files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("deps")
    for f in fn
    if f.endswith(".json")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run_json, files)
