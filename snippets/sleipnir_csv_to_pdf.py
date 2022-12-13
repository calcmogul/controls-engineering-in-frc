#!/usr/bin/env python3

"""Generates plot PDFs from Sleipnir CSV files."""

import re
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

from bookutil import latex

if "--noninteractive" in sys.argv:
    mpl.use("svg")
plt.rc("text", usetex=True)


def main():
    """Entry point."""
    filenames = sys.argv[1:]

    for filename in filenames:
        with open(filename, encoding="utf-8") as f:
            labels = f.readline().rstrip().split(",")
        data = np.genfromtxt(filename, delimiter=",", skip_header=1)

        for i in range(data.shape[1] - 1):
            label = labels[i + 1]

            plt.figure()
            ax = plt.gca()
            ax.plot(data[:, :1], data[:, i + 1 : i + 2], label=label)
            ax.set_xlabel(labels[0])
            ax.set_ylabel(label)
            ax.legend()

            figname = re.search(r"(\w+)(\(.*?\))?", label).group(1)
            latex.savefig(f"{filename.removesuffix('.csv')}{figname}")


if __name__ == "__main__":
    main()
