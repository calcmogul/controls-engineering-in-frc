#!/usr/bin/env python3

"""Ensure URLs in every .tex file are reachable."""

import multiprocessing as mp
import os
import re
import sys

import requests
import urllib3


def lint_links(link):
    """Runs the URL in the regex Match object through the link linter.

    Keyword arguments:
    link -- a tuple containing the filename, line number of URL, and URL
    """
    return verify_url(link[0], link[1], link[2])


def verify_url(filename, line_number, url):
    """Verifies URL is reachable and returns 200 status code.

    Keyword arguments:
    filename -- name of file containing URL
    line_number -- line number of URL
    url -- the URL to verify

    Returns:
    True if verification succeeded or False otherwise
    """
    try:
        headers = {"User-Agent": "Mozilla/5.0"}
        try:
            r = requests.head(url, headers=headers, timeout=5)
        except requests.exceptions.SSLError:
            urllib3.disable_warnings()
            r = requests.head(url, headers=headers, timeout=5, verify=False)

        # researchgate.net tends to block scripts, so don't return verification
        # failure for its links
        if r.status_code == 403 and url.startswith("https://www.researchgate.net"):
            return True

        # motors.vex.com tends to block scripts, so don't return verification
        # failure for its links
        if r.status_code == 403 and url.startswith("https://motors.vex.com"):
            return True

        # Allow redirects for YouTube shortlinks
        if r.status_code == 303 and url.startswith("https://youtu.be"):
            return True

        if r.status_code != 200:
            print(f"[{filename}:{line_number}]\n    {url}\n    {r.status_code}")
            if url != r.url:
                print(f"    redirected to {r.url}")
            return False
    except requests.ConnectionError as ex:
        print(f"[{filename}:{line_number}]\n    {url}\n    {ex}")
        return False
    except requests.exceptions.Timeout as ex:
        # motors.vex.com tends to time out for scripts, so don't return
        # verification failure for its links
        if url.startswith("https://motors.vex.com"):
            return True

        print(f"[{filename}:{line_number}]\n    {url}\n    {ex}")
        return False
    return True


# commit-hash.tex is ignored because it may reference a local commit that hasn't
# yet been pushed. In that case, the GitHub URL for it wouldn't yet exist.
files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if (f.endswith(".tex") or f.endswith(".bib"))
    and not f.endswith("commit-hash.tex")
    and "build/venv/" not in dp
]

cmd_rgx = re.compile(r"\\(url|href){(?P<url>[^}]+)}")
bib_rgx = re.compile(r"url\s*=\s*{(?P<url>[^}]+)}")

# link tuples contain:
#   filename -- filename
#   contents -- file contents
#   match -- regex Match object
links = []
for file in files:
    # Get file contents
    with open(file, "r", encoding="utf-8") as f:
        contents = f.read()

    for match in list(cmd_rgx.finditer(contents)) + list(bib_rgx.finditer(contents)):
        # Get line regex match was on
        linecount = 1
        for i in range(match.start()):
            if contents[i] == os.linesep:
                linecount += 1

        links.append((file, linecount, match.group("url")))

with mp.Pool(mp.cpu_count()) as pool:
    results = pool.map(lint_links, links)

if all(results):
    sys.exit(0)
else:
    sys.exit(1)
