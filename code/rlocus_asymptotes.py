#!/usr/bin/env python3

import control as cnt
from frccontrol import conv
import matplotlib.pyplot as plt

#              1
# G(s) = --------------
#        (s + 2)(s + 3)
G = cnt.tf(1, conv([1, 0], [1, 2], [1, 3]))
cnt.root_locus(G)

plt.title("Root Locus")
plt.xlabel("Real Axis (seconds$^{-1}$)")
plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
plt.savefig("rlocus_asymptotes.svg")
