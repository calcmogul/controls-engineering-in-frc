#!/usr/bin/env python3

import multiprocessing as mp
import os
import subprocess
import sys

sys.path.insert(0, os.getcwd())
from deputils import fetch_git_dependency


# Create venv
if not os.path.exists("build/venv"):
    subprocess.run([sys.executable, "-m", "venv", "build/venv"])

# venv executables
root = os.getcwd()
ENV_PYTHON = os.path.join(root, "build/venv/bin/python3")
ENV_PIP = os.path.join(root, "build/venv/bin/pip3")


def run(name):
    subprocess.run([ENV_PYTHON, name, "--save-plots", "--noninteractive"])
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


# Set up control (dep of frccontrol)
os.chdir(root)
fetch_git_dependency(
    repo="git://github.com/python-control/python-control", commit="0.8.2"
)
os.chdir("build/python-control")
subprocess.run(
    [
        "git",
        "apply",
        os.path.join(
            root,
            "patches/0001-Don-t-remove-unobservable-or-uncontrollable-states-f.patch",
        ),
    ]
)
subprocess.run([ENV_PIP, "install", "-e", "."])
os.chdir(root)

# Set up frccontrol
fetch_git_dependency(
    repo="git://github.com/calcmogul/frccontrol",
    commit="e7364a04420a44fb8137740afff1a3dd0e4a5ad6",
)
os.chdir("build/frccontrol")
subprocess.run([ENV_PIP, "install", "-e", "."])
os.chdir(root)

# Run frccontrol examples
os.chdir("build/frccontrol/examples")
files = [
    os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn if f.endswith(".py")
]
with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
