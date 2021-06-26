#!/usr/bin/env python3

import os


def proc_init(python_exec_copy):
    global python_exec
    python_exec = python_exec_copy


def run(name):
    import bookutil.latex as latex
    import subprocess

    subprocess.run([python_exec, name, "--noninteractive"])
    filename = os.path.splitext(os.path.basename(name))[0] + "_response"
    latex.convert_svg2pdf(filename)


def main():
    import multiprocessing as mp

    # Run frccontrol examples
    old_cwd = os.getcwd()
    python_exec = os.path.join(old_cwd, "build/venv/bin/python3")
    os.chdir("build/frccontrol/examples")
    files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk(".")
        for f in fn
        if f.endswith(".py")
    ]
    with mp.Pool(mp.cpu_count(), proc_init, (python_exec,)) as pool:
        pool.map(run, files)
    os.chdir(old_cwd)


if __name__ == "__main__":
    main()
