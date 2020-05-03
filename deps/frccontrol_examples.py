#!/usr/bin/env python3

import multiprocessing as mp
import os


root = os.getcwd()
ENV_PYTHON = os.path.join(root, "build/venv/bin/python3")
ENV_PIP = os.path.join(root, "build/venv/bin/pip3")


def run(name):
    import bookutil.latex as latex
    import subprocess

    subprocess.run([ENV_PYTHON, name, "--noninteractive"])
    filename = os.path.splitext(os.path.basename(name))[0] + "_response"
    latex.convert_svg2pdf(filename)


# Run frccontrol examples
os.chdir("build/frccontrol/examples")
files = [
    os.path.join(dp, f) for dp, dn, fn in os.walk(".") for f in fn if f.endswith(".py")
]
with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run, files)
