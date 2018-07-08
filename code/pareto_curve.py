#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

plt.rc("text", usetex=True)


def main():
    x_axis = np.arange(1 / 3, 3, 0.001)
    plt.plot(x_axis, [1 / x for x in x_axis], label="LQR")
    plt.xlabel("$\Vert u^TRu \Vert_2$")
    plt.ylabel("$\Vert x^TQx \Vert_2$")
    plt.xticks([])
    plt.yticks([])
    plt.legend()
    plt.savefig("pareto_curve.svg")


if __name__ == "__main__":
    main()
