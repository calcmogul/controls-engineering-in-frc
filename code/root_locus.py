#!/usr/bin/env python3

import control as cnt
import matplotlib.pyplot as plt
import numpy as np

plt.rc('text', usetex=True)


# Implementation of MATLAB's conv() function
def conv(list1, list2):
    return np.convolve(list1, list2).tolist()


# Create transfer function and plot root locus
G = cnt.tf(conv([1, -9 + 9j], [1, -9 - 9j]), conv([1, 0], [1, 10]))
cnt.root_locus(G)

# Show plot
plt.title("Root Locus")
plt.xlabel("Real Axis (seconds$^{-1}$)")
plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
plt.gca().set_aspect("equal")
plt.savefig("root_locus.svg")
