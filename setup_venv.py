#!/usr/bin/env python3

"""Sets up the book's Python virtual environment."""

import platform
import subprocess
import sys
from pathlib import Path


def main():
    """Entry point."""
    venv_name = Path("build/venv")

    # Create venv
    if not venv_name.exists():
        subprocess.run([sys.executable, "-m", "venv", venv_name], check=True)
        if platform.system() == "Windows":
            ENV_PIP = Path.cwd() / venv_name / "Scripts" / "pip3"
        else:
            ENV_PIP = Path.cwd() / venv_name / "bin" / "pip3"
        subprocess.run([ENV_PIP, "install", "wheel"], check=True)


if __name__ == "__main__":
    main()
