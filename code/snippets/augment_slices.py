#!/usr/bin/env python3

import numpy as np


def main():
    A = np.array([[1, 2], [3, 4]])
    B = np.array([[5], [6]])
    C = np.array([[7, 8]])
    D = np.array([[9]])

    tmp = np.zeros((3, 3))
    tmp[0:2, 0:2] = A
    tmp[0:2, 2:3] = B
    tmp[2:3, 0:2] = C
    tmp[2:3, 2:3] = D

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
