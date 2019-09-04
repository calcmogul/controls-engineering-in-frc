#!/usr/bin/env python3

import multiprocessing as mp
import os
import subprocess


root = os.getcwd()
ENV_PYTHON = os.path.join(root, "build/venv/bin/python3")
ENV_PIP = os.path.join(root, "build/venv/bin/pip3")


def run(name):
    subprocess.run([ENV_PYTHON, name, "--noninteractive"])
    base = os.path.splitext(os.path.basename(name))[0]
    for suffix in [
        "pzmap_open-loop",
        "pzmap_closed-loop",
        "pzmap_observer",
        "response",
    ]:
        subprocess.run(
            [
                "inkscape",
                "-D",
                "-z",
                "--file=" + base + "_" + suffix + ".svg",
                "--export-pdf=" + base + "_" + suffix + ".pdf",
            ]
        )


# Run frccontrol examples
os.chdir("build/frccontrol/examples")
files = [
    os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn if f.endswith(".py")
]
with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
