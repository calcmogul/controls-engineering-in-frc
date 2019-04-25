#!/usr/bin/env python3

import multiprocessing as mp
import os
import subprocess
import sys

sys.path.insert(0, os.getcwd())
from deputils import fetch_git_dependency


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


fetch_git_dependency(
    repo="git://github.com/calcmogul/frccontrol",
    commit="e7364a04420a44fb8137740afff1a3dd0e4a5ad6",
)
os.chdir("examples")

files = [
    os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn if f.endswith(".py")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
