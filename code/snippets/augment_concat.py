#!/usr/bin/env python3

import numpy as np


def main():
    A = np.array([[1, 2], [3, 4]])
    B = np.array([[5], [6]])
    C = np.array([[7, 8]])
    D = np.array([[9]])

    tmp = np.concatenate(
        (np.concatenate((A, B), axis=1), np.concatenate((C, D), axis=1)), axis=0
    )

    print("A =")
    print(A)
    print("B =")
    print(B)
    print("C =")
    print(C)
    print("D =")
    print(D)
    print("[A, B; C, D] =")
    print(tmp)


if __name__ == "__main__":
    main()
