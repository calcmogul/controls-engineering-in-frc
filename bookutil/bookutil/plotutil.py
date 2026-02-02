"""Plotting utilities."""

from matplotlib import animation


def plot_xy(fig, ref_xs, ref_ys, state_xs, state_ys, patches=[]):
    """
    Plot x-y data.

    Parameter ``fig``:
        Figure on which to plot.

    Parameter ``ref_xs``:
        List of reference x values.

    Parameter ``ref_ys``:
        List of reference y values.

    Parameter ``state_xs``:
        List of state x values.

    Parameter ``state_ys``:
        List of state y values.

    Parameter ``patches``:
        List of patches to draw (default: []).
    """
    ax = fig.add_subplot()

    ref = ax.plot(ref_xs, ref_ys, label="Reference")[0]
    state = ax.plot(state_xs, state_ys, label="State")[0]

    for patch in patches:
        ax.add_patch(patch)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_aspect(1.0)
    ax.set_box_aspect(1.0)

    return ref, state


def animate_xy(fig, ref_xs, ref_ys, state_xs, state_ys, dt, patches=[]):
    """
    Plot x-y data and animate it.

    Parameter ``fig``:
        Figure on which to plot.

    Parameter ``ref_xs``:
        List of reference x values.

    Parameter ``ref_ys``:
        List of reference y values.

    Parameter ``state_xs``:
        List of state x values.

    Parameter ``state_ys``:
        List of state y values.

    Parameter ``dt``:
        Timestep duration.

    Parameter ``patches``:
        List of patches to draw.
    """
    ref, state = plot_xy(fig, ref_xs, ref_ys, state_xs, state_ys, patches)

    def animate(i):
        ref.set_data(ref_xs[: i + 1], ref_ys[: i + 1])
        state.set_data(state_xs[: i + 1], state_ys[: i + 1])

        return ref, state

    return animation.FuncAnimation(
        fig,
        animate,
        frames=len(state_xs),
        interval=int(dt * 1000),
        blit=True,
        repeat=True,
    )
