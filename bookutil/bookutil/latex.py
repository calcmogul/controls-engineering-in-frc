"""Utility functions for LaTeX figures."""

import re
import subprocess

import matplotlib.pyplot as plt


def convert_svg2pdf(filename):
    """Converts the given SVG to PDF with inkscape.

    Keyword arguments:
    filename -- filename without the extension
    """
    inkscape_output = subprocess.check_output(
        ["inkscape", "--version"], stderr=subprocess.DEVNULL, encoding="utf-8"
    )
    match = re.search(r"(?P<major>[0-9]+)\.(?P<minor>[0-9]+)", inkscape_output)

    # Inkscape 0.x has different arguments than later versions
    if match.group("major") == "0":
        subprocess.run(
            [
                "inkscape",
                "-D",
                "-z",
                "--file=" + filename + ".svg",
                "--export-pdf=" + filename + ".pdf",
            ],
            check=True,
        )
    else:
        subprocess.run(
            [
                "inkscape",
                "-D",
                "--export-type=pdf",
                "--export-filename=" + filename + ".pdf",
                filename + ".svg",
            ],
            check=True,
        )


def savefig(filename):
    """Saves the current plot as an SVG and converts it to a PDF with inkscape.

    Keyword arguments:
    filename -- filename without the extension
    """
    plt.savefig(filename + ".svg", bbox_inches="tight")
    convert_svg2pdf(filename)
