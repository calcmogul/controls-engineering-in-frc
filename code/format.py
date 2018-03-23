#!/usr/bin/env python3

import os
import subprocess


def main():
    files = [
        f for f in os.listdir("code") if os.path.isfile(f) and f.endswith(".py")
    ]

    for f in files:
        subprocess.run(["python3", "-m", "yapf", "--style", "google", "-i", f])


if __name__ == "__main__":
    main()
