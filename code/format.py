#!/usr/bin/env python3

import os
import subprocess


def main():
    files = [
        os.path.join(dp, f)
        for dp, dn, fn in os.walk("code")
        for f in fn
        if f.endswith(".py")
    ]

    for f in files:
        subprocess.run(["python3", "-m", "yapf", "--style", "google", "-i", f])


if __name__ == "__main__":
    main()
