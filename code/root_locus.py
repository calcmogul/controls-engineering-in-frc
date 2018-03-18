#!/usr/bin/env python3

import control
from matplotlib import pyplot
import numpy

pyplot.rc('text', usetex=True)


# Implementation of MATLAB's conv() function
def conv(list1, list2):
    return numpy.convolve(list1, list2).tolist()


# Create transfer function and plot root locus
G = control.tf(conv([1, -9 + 9j], [1, -9 - 9j]), conv([1, 0], [1, 10]))
control.root_locus(G)

# Show plot
pyplot.title("Root Locus")
pyplot.xlabel("Real Axis (seconds$^{-1}$)")
pyplot.ylabel("Imaginary Axis (seconds$^{-1}$)")
pyplot.gca().set_aspect("equal")
pyplot.savefig("root_locus.svg")
