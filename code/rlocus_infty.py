#!/usr/bin/env python3

import matplotlib as mpl
mpl.use("svg")
import control as cnt
import matplotlib.pyplot as plt

import latexutils

#          1
# G(s) = -----
#        s - 1
G = cnt.tf([1], [1, -1])
cnt.root_locus(G)

plt.title("Root Locus")
plt.xlabel("Real Axis (seconds$^{-1}$)")
plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
latexutils.savefig("rlocus_infty")
