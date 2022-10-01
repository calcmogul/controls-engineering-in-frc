#!/usr/bin/env python3

"""Sets up the book's Python virtual environment."""

import os
import subprocess
import sys


def main():
    """Entry point."""
    venv_name = "build/venv"

    # Create venv
    if not os.path.exists(venv_name):
        subprocess.run([sys.executable, "-m", "venv", venv_name], check=True)
        ENV_PIP = os.path.join(os.getcwd(), f"{venv_name}/bin/pip3")
        subprocess.run([ENV_PIP, "install", "wheel"], check=True)


if __name__ == "__main__":
    main()
