#!/usr/bin/env python3

import os
import subprocess
import sys


def main():
    venv_name = "build/venv"

    # Create venv
    if not os.path.exists(venv_name):
        subprocess.run([sys.executable, "-m", "venv", venv_name])
        ENV_PIP = os.path.join(os.getcwd(), f"{venv_name}/bin/pip3")
        subprocess.run([ENV_PIP, "install", "wheel"])


if __name__ == "__main__":
    main()
