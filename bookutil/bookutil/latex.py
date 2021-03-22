import matplotlib as mpl
import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def plot_time_responses(
    system, t, x_rec, ref_rec, u_rec, ndigits, title=None, use_pid_labels=False
):
    """Plots time-domain responses for a given system with the K_p and K_d
    controller gains included in the legend.

    Keyword arguments:
    time -- list of timesteps corresponding to references
    x_rec -- recording of state estimates from generate_time_responses()
    ref_rec -- recording of references from generate_time_responses()
    u_rec -- recording of inputs from generate_time_responses()
    ndigits -- number of digits after decimal point to include in gains
    title -- title for time-domain plots (default: no title)
    use_pid_labels -- whether to use PID controller or state-space controller
                      labels (output and setpoint vs state and reference)
    """
    plt.figure()
    subplot_max = system.sysd.nstates + system.sysd.ninputs
    for i in range(system.sysd.nstates):
        plt.subplot(subplot_max, 1, i + 1)
        if system.sysd.nstates + system.sysd.ninputs > 3:
            plt.ylabel(
                system.state_labels[i],
                horizontalalignment="right",
                verticalalignment="center",
                rotation=45,
            )
        else:
            plt.ylabel(system.state_labels[i])
        if use_pid_labels:
            label = "Output"
        else:
            label = "State"
        if i == 0:
            plt.title(title)
            label += f" ($K_p = {round(system.K[0, 0], ndigits)}$)"
        elif i == 1:
            label += f" ($K_d = {round(system.K[0, 1], ndigits)}$)"
        plt.plot(t, system.extract_row(x_rec, i), label=label)
        if use_pid_labels:
            label = "Setpoint"
        else:
            label = "Reference"
        plt.plot(t, system.extract_row(ref_rec, i), label=label)
        plt.legend()

    for i in range(system.sysd.ninputs):
        plt.subplot(subplot_max, 1, system.sysd.nstates + i + 1)
        if system.sysd.nstates + system.sysd.ninputs > 3:
            plt.ylabel(
                system.u_labels[i],
                horizontalalignment="right",
                verticalalignment="center",
                rotation=45,
            )
        else:
            plt.ylabel(system.u_labels[i])
        plt.plot(t, system.extract_row(u_rec, i), label="Control effort")
        plt.legend()
    plt.xlabel("Time (s)")


def convert_svg2pdf(filename):
    """Converts the given SVG to PDF with inkscape.

    Keyword arguments:
    filename -- filename without the extension
    """
    import re
    import subprocess

    inkscape_output = subprocess.check_output(
        ["inkscape", "--version"], encoding="utf-8"
    )
    match = re.search(r"(?P<major>[0-9]+)\.(?P<minor>[0-9]+)", inkscape_output)

    # Inkscape 0.x has different arguments than later versions
    if match.group("major") == "0":
        subprocess.run(
            [
                "inkscape",
                "-D",
                "-z",
                "--file=" + filename + ".svg",
                "--export-pdf=" + filename + ".pdf",
            ]
        )
    else:
        subprocess.run(
            [
                "inkscape",
                "-D",
                "--export-type=pdf",
                "--export-filename=" + filename + ".pdf",
                filename + ".svg",
            ]
        )


def savefig(filename):
    """Saves the current plot as an SVG and converts it to a PDF with inkscape.

    Keyword arguments:
    filename -- filename without the extension
    """
    plt.savefig(filename + ".svg")
    convert_svg2pdf(filename)
