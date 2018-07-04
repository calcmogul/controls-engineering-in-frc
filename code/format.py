#!/usr/bin/env python3

import subprocess
import sys

if len(sys.argv) > 1:
    for f in sys.argv[1:]:
        cmd = [sys.executable, "-m", "yapf", "--style", "google", "-i", f]
        subprocess.run(cmd)
