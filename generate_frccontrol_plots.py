#!/usr/bin/env python3

import multiprocessing as mp
import os
import subprocess
import sys


def run(name):
    subprocess.run([sys.executable, name, "--save-plots", "--noninteractive"])
    base = os.path.splitext(os.path.basename(name))[0]
    for suffix in ["pzmaps", "response"]:
        subprocess.run(
            [
                "inkscape",
                "-D",
                "-z",
                "--file=" + base + "_" + suffix + ".svg",
                "--export-pdf=" + base + "_" + suffix + ".pdf",
            ]
        )

os.chdir("build")

files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("frccontrol/examples")
    for f in fn
    if f.endswith(".py")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
