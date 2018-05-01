"""Control system utility functions."""

import control as cnt
import matplotlib.pyplot as plt


def dpzmap(sys, title):
    """Plot poles and zeroes of discrete system.

    Keyword arguments:
    sys -- the system to plot
    title -- the title of the plot
    """
    cnt.pzmap(sys, title=title)
    circle = plt.Circle((0, 0), radius=1, fill=False)
    ax = plt.gca()
    ax.add_artist(circle)
    plt.xlim([-1, 1])
    plt.ylim([-1, 1])
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    ax.set_aspect(abs(x1 - x0) / abs(y1 - y0))


def ss_closed_loop(sys, K):
    """Constructs the closed-loop system given a controller K."""
    return cnt.StateSpace(sys.A - sys.B * K, sys.B, sys.C, sys.D)
