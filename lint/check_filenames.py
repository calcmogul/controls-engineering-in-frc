#!/usr/bin/env python3

"""
Ensure .tex filenames use hyphens and .py files use underscores. Vice versa is
not allowed.
"""

import sys
from pathlib import Path


def main():
    error_occurred = False
    for f in [
        f
        for f in Path(".").rglob("*")
        if f.suffix in [".tex", ".bib"] and not f.is_relative_to("./build")
    ]:
        if "_" in f.name:
            print(f"error: filename '{f}' should not include underscores")
            error_occurred = True

    for f in [
        f
        for f in Path(".").rglob("*")
        if f.suffix == ".py" and not f.is_relative_to("./build")
    ]:
        if "-" in f.name:
            print(f"error: filename '{f}' should not include hyphens")
            error_occurred = True

    if error_occurred:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()
