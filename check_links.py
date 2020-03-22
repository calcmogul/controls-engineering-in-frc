#!/usr/bin/env python3

"""Ensure URLs in every .tex file are reachable"""

import os
import re
import requests
import sys


def verify_url(filename, file_contents, match, url):
    """Verifies URL is reachable and returns 200 status code.

    Keyword arguments:
    filename -- name of file containing URL
    file_contents -- file contents
    match -- the regex Match object for the URL
    url -- the URL to verify

    Returns:
    True if verification succeeded or False otherwise
    """
    try:
        r = requests.head(url)
        if r.status_code != 200:
            # Get line regex match was on
            linecount = 1
            for i in range(match.start()):
                if file_contents[i] == os.linesep:
                    linecount += 1
            print(f"[{filename}:{linecount}] error: {url} returned {r.status_code}")
            return False
    except requests.ConnectionError as ex:
        print(f"[{filename}] warning: {url} {str(ex)}")
        return False
    return True


cmd_rgx = re.compile(
    r"""
    \\(?P<command>[a-z]+)\*?       # Commaned name
    ({(?P<arg1>[^}]+)}             # First argument
     (?P<arg_count>\[[0-9]+\])?)?  # Optional square bracket arg count
    ({(?P<arg2>[^}]+)})?""",
    re.VERBOSE | re.MULTILINE,
)
bib_rgx = re.compile(r"url\s*=\s*{(?P<url>[^}]+)}")

files = [
    os.path.join(dp, f)[2:]
    for dp, dn, fn in os.walk(".")
    for f in fn
    if (f.endswith(".tex") or f.endswith(".bib")) and "build/venv/" not in dp
]

success = True
for filename in files:
    # Get file contents
    with open(filename, "r") as f:
        contents = f.read()

    for match in cmd_rgx.finditer(contents):
        if match.group("command") in ["url", "href"]:
            url = match.group("arg1")
            success &= verify_url(filename, contents, match, url)

    for match in bib_rgx.finditer(contents):
        url = match.group("url")
        success &= verify_url(filename, contents, match, url)

if success:
    sys.exit(0)
else:
    sys.exit(1)
