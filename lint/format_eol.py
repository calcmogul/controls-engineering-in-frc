#!/usr/bin/env python3

"""Removes trailing whitespace from .tex files."""

from pathlib import Path


def main():
    files: list[Path] = [
        f
        for f in Path(".").rglob("*")
        if f.suffix == ".tex" and not f.is_relative_to("./build")
    ]

    for file in files:
        with open(file, encoding="utf-8") as f:
            lines = f.read()

        output = "".join(line.rstrip() + "\n" for line in lines.splitlines())
        if lines != output:
            with open(file, "w", encoding="utf-8") as f:
                f.write(output)


if __name__ == "__main__":
    main()
