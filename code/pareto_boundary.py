#!/usr/bin/env python3

import matplotlib as mpl
mpl.use("svg")
import matplotlib.pyplot as plt
import numpy as np

import latexutils

plt.rc("text", usetex=True)


def main():
    x = np.arange(1 / 3, 3, 0.001)
    y1 = [1 / val for val in x]
    y2 = [3 for val in x]
    plt.plot(x, y1, label="LQR")
    plt.fill_between(
        x, y1, y2, color=(1, 0.5, 0.05), alpha=0.5, label="Pole placement")
    plt.xlabel("$\Vert u^TRu \Vert_2$")
    plt.ylabel("$\Vert x^TQx \Vert_2$")
    plt.xticks([])
    plt.yticks([])
    plt.legend()
    latexutils.savefig("pareto_boundary")


if __name__ == "__main__":
    main()
