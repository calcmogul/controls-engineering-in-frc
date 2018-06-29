#!/usr/bin/env python3

import control as cnt
from frccontrol import conv
import matplotlib.pyplot as plt

#        (s - 9 + 9i)(s - 9 - 9i)
# G(s) = ------------------------
#               s(s + 10)
G = cnt.tf(conv([1, -9 + 9j], [1, -9 - 9j]), conv([1, 0], [1, 10]))
cnt.root_locus(G)

# Show plot
plt.title("Root Locus")
plt.xlabel("Real Axis (seconds$^{-1}$)")
plt.ylabel("Imaginary Axis (seconds$^{-1}$)")
plt.gca().set_aspect("equal")
plt.savefig("rlocus_zeroes.svg")
