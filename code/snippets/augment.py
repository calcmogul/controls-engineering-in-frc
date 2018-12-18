#!/usr/bin/env python3

import numpy as np


def main():
    J = 7.7500e-05
    b = 8.9100e-05
    Kt = 0.0184
    Ke = 0.0211
    R = 0.0916
    L = 5.9000e-05

    # fmt: off
    A = np.array([[-b / J, Kt / J],
                  [-Ke / L, -R / L]])
    B = np.array([[0],
                  [1 / L]])
    C = np.array([[1, 0]])
    D = np.array([[0]])
    # fmt: on

    print("A =")
    print(A)
    print("B =")
    print(B)
    print("C =")
    print(C)
    print("D =")
    print(D)

    tmp = np.concatenate(
        (
            np.concatenate((A - np.eye(A.shape[0]), B), axis=1),
            np.concatenate((C, D), axis=1),
        ),
        axis=0,
    )

    print("[A, B; C, D] =")
    print(tmp)


if __name__ == "__main__":
    main()
