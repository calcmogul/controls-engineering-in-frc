"""Plotting utilities."""

from matplotlib import animation


def plot_xy(fig, ref_xs, ref_ys, state_xs, state_ys):
    """
    Plot x-y data.

    Keyword arguments:
    fig -- figure on which to plot
    ref_xs -- list of reference x values
    ref_ys -- list of reference y values
    state_xs -- list of state x values
    state_ys -- list of state y values
    """
    ax = fig.add_subplot()

    ref = ax.plot(ref_xs, ref_ys, label="Reference")[0]
    state = ax.plot(state_xs, state_ys, label="State")[0]

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_aspect(1.0)
    ax.set_box_aspect(1.0)

    return ref, state


def animate_xy(fig, ref_xs, ref_ys, state_xs, state_ys, dt):
    """
    Plot x-y data and animate it.

    Keyword arguments:
    fig -- figure on which to plot
    ref_xs -- list of reference x values
    ref_ys -- list of reference y values
    state_xs -- list of state x values
    state_ys -- list of state y values
    dt -- timestep duration
    """
    ref, state = plot_xy(fig, ref_xs, ref_ys, state_xs, state_ys)

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
