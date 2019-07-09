#!/usr/bin/env python3

# Avoid needing display if plots aren't being shown
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
    import utils.latex as latex

import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def main():
    x = np.arange(1 / 3, 3, 0.001)
    y1 = [1 / val for val in x]
    y2 = [3 for val in x]
    plt.plot(x, y1, label="LQR")
    plt.fill_between(x, y1, y2, color=(1, 0.5, 0.05), alpha=0.5, label="Pole placement")
    plt.xlabel("$\Vert u^TRu \Vert_2$")
    plt.ylabel("$\Vert x^TQx \Vert_2$")
    plt.xticks([])
    plt.yticks([])
    plt.legend()
    if "--noninteractive" in sys.argv:
        latex.savefig("pareto_boundary")
    else:
        plt.show()


if __name__ == "__main__":
    main()
