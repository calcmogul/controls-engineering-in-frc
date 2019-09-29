#!/usr/bin/env python3

import json
import os
import shutil
import subprocess
import sys


class Package:
    def __init__(self, metadata):
        """Create a Package instance from package metadata.

        Keyword arguments:
        metadata -- package metadata
        """
        # repo -- URL of Git repository
        # commit -- commit to fetch from Git repository
        self.name = metadata["name"]
        self.__depends = metadata["depends"]
        self.repo = metadata["repo"]
        self.commit = metadata["commit"]
        self.posthooks = metadata["posthooks"]
        self.parents = []
        self.children = []

    def has_direct_dependency(self, name):
        """Returns true if name of package given is a direct dependency of this
        one.

        Keyword arguments:
        name -- name of possibly dependent package
        """
        return name in self.__depends


class PackageTree:
    def __init__(self, filename):
        """Construct and install a package dependency tree

        Keyword arguments:
        filename -- name of json file containing package descriptors
        """
        # Read JSON file containing package descriptors
        with open(filename) as input:
            descs = json.load(input)
        pkgs = []
        for desc in descs:
            pkgs.append(Package(desc))

        # Build trees of packages based on dependencies. The "trees" list only
        # contains root packages (packages with no parents).
        self.trees = []
        for parent_pkg in pkgs:
            for maybe_child in pkgs:
                if parent_pkg.has_direct_dependency(maybe_child.name):
                    maybe_child.parents.append(parent_pkg)
                    parent_pkg.children.append(maybe_child)
        for pkg in pkgs:
            if len(pkg.parents) == 0:
                self.trees.append(pkg)

    def install_pkgs(self, venv_name):
        """Install packages starting from dependencies first.

        Keyword arguments:
        venv_name -- location and name of venv

        Returns:
        env_python -- python executable associated with venv
        env_pip -- pip executable associated with venv
        """
        # Create venv
        if not os.path.exists(venv_name):
            subprocess.run([sys.executable, "-m", "venv", venv_name])

        # venv executables
        self.root = os.getcwd()
        self.ENV_PYTHON = os.path.join(self.root, f"{venv_name}/bin/python3")
        self.ENV_PIP = os.path.join(self.root, f"{venv_name}/bin/pip3")

        for tree in self.trees:
            self.__install_pkg(tree)

        return self.ENV_PYTHON, self.ENV_PIP

    def __install_pkg(self, pkg):
        """Install package.

        Keyword arguments:
        pkg -- package
        """
        # Install dependencies first
        for child in pkg.children:
            self.__install_pkg(child)

        # If repository exists, verify desired commit is checked out. Otherwise,
        # reclone to retreive desired commit.
        if os.path.exists(f"build/{pkg.name}"):
            os.chdir(f"build/{pkg.name}")
            commit = subprocess.check_output(
                ["git", "rev-parse", "HEAD"], encoding="utf-8"
            ).rstrip()
            os.chdir(self.root)

            if pkg.commit == commit:
                # Package is already installed
                return
            else:
                # Remove repository
                shutil.rmtree(f"build/{pkg.name}", ignore_errors=True)

        os.makedirs(f"build/{pkg.name}")

        # Create new Git repository
        os.chdir(f"build/{pkg.name}")
        subprocess.run(["git", "init", "--quiet"])
        subprocess.run(["git", "remote", "add", "origin", pkg.repo])

        # Fetch master first since desired commit is on master and GitHub
        # doesn't allow fetches of specific commits that aren't pointed to by a
        # ref
        subprocess.run(["git", "fetch", "--quiet", "origin", "master"])

        # Checkout the specified commit and install it
        subprocess.run(["git", "checkout", "--quiet", pkg.commit])
        subprocess.run([self.ENV_PIP, "install", "-e", "."])
        os.chdir(self.root)

        # Run post-transaction hooks
        for hook in pkg.posthooks:
            subprocess.run([self.ENV_PYTHON, hook])


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("deps")
    for f in fn
    if f.endswith(".json")
]
for file in files:
    pkgs = PackageTree(file)
    pkgs.install_pkgs("build/venv")
