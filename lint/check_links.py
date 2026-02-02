#!/usr/bin/env python3

"""Ensure URLs in every .tex file are reachable."""

import multiprocessing as mp
import os
import re
import sys
from pathlib import Path

import requests
import urllib3


def lint_links(link):
    """
    Runs the URL in the regex Match object through the link linter.

    Parameter ``link``:
        A tuple containing the filename, line number of URL, and URL.
    """
    return verify_url(link[0], link[1], link[2])


def verify_url(filename, line_number, url):
    """
    Verifies URL is reachable and returns 200 status code.

    Parameter ``filename``:
        Name of file containing URL.

    Parameter ``line_number``:
        Line number of URL.

    Parameter ``url``:
        The URL to verify.

    Returns:
        True if verification succeeded or False otherwise.
    """
    try:
        headers = {"User-Agent": "Mozilla/5.0"}
        try:
            r = requests.head(url, headers=headers, timeout=5)
        except requests.exceptions.SSLError:
            urllib3.disable_warnings()
            r = requests.head(url, headers=headers, timeout=5, verify=False)

        # These links tend to block scripts, so don't return verification
        # failure for them
        if r.status_code == 403 and url.startswith("https://ethw.org"):
            return True

        # Allow redirects for YouTube shortlinks
        if r.status_code == 303 and url.startswith("https://youtu.be"):
            return True

        if r.status_code != 200:
            print(f"[{filename}:{line_number}]\n    {url}\n    {r.status_code}")
            if url != r.url:
                print(f"    redirected to {r.url}")
            return False
    except (requests.ConnectionError, requests.exceptions.Timeout) as ex:
        print(f"[{filename}:{line_number}]\n    {url}\n    {ex}")
        return False
    return True


def main():
    # commit-hash.tex is ignored because it may reference a local commit that
    # hasn't yet been pushed. In that case, the GitHub URL for it wouldn't yet
    # exist.
    files: list[Path] = [
        f
        for f in Path(".").rglob("*")
        if (f.suffix in [".tex", ".bib"])
        and f.name != "commit-hash.tex"
        and not f.is_relative_to("./build/venv")
    ]

    cmd_rgx = re.compile(r"\\(url|href){(?P<url>[^}]+)}")
    bib_rgx = re.compile(r"url\s*=\s*{(?P<url>[^}]+)}")

    # link tuples contain:
    #   filename -- filename
    #   contents -- file contents
    #   match -- regex Match object
    links = []
    for file in files:
        contents = file.read_text(encoding="utf-8")

        for match in list(cmd_rgx.finditer(contents)) + list(
            bib_rgx.finditer(contents)
        ):
            # Get line regex match was on
            linecount = 1
            for i in range(match.start()):
                if contents[i] == os.linesep:
                    linecount += 1

            # "\" replacement removes LaTeX escapes
            links.append(
                (file.as_posix(), linecount, match.group("url").replace("\\", ""))
            )

    with mp.Pool(mp.cpu_count()) as pool:
        results = pool.map(lint_links, links)

    if all(results):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
