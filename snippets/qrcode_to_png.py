#!/usr/bin/env python3

"""Generates QR code PNGs from URLs."""

import os
import sys
import urllib.parse

import qrcode


def main():
    """Entry point."""
    try:
        includegraphics_args = sys.argv[1]
        url = sys.argv[2]

        filename = (
            # Add directory prefix
            os.path.join(
                "build",
                # Add filename prefix
                "qrcode_" +
                # Normalize URL
                urllib.parse.quote(url)
                # Replace invalid filesystem path characters
                .replace(":", "_").replace(".", "_").replace("/", "_")
                # Replace invalid LaTeX characters
                .replace("%", "_"),
            )
            # Add file extension
            + ".png"
        )

        # Write PNG to file and print LaTeX for rendering it
        qrcode.make(url).save(filename)
        print(f"\\includegraphics[{includegraphics_args}]" + "{" + filename + "}")

        # Flush output to force SIGPIPE to be triggered here
        sys.stdout.flush()
    except BrokenPipeError:
        # Python flushes standard streams on exit; redirect remaining output
        # to devnull to avoid another BrokenPipeError at shutdown
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())


if __name__ == "__main__":
    main()
