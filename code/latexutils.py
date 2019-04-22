import matplotlib as mpl

mpl.use("svg")
import matplotlib.pyplot as plt

plt.rc("text", usetex=True)


def savefig(file_name):
    """Saves the current plot as an SVG and converts it to a PDF with inkscape.

    Keyword arguments:
    file_name -- file name without the extension
    """
    import subprocess

    plt.savefig(file_name + ".svg")
    subprocess.run(
        [
            "inkscape",
            "-D",
            "-z",
            "--file=" + file_name + ".svg",
            "--export-pdf=" + file_name + ".pdf",
        ]
    )
