import os
import shutil
import subprocess


def fetch_git_dependency(repo, commit):
    """Fetches the specified commit from a remote Git repository and places it
    in the build folder.

    Keyword arguments:
    repo -- URL of Git repository
    commit -- commit to fetch from Git repository
    """
    name = os.path.basename(repo)

    # Remove repository if it exists
    if os.path.exists(f"build/{name}"):
        shutil.rmtree(f"build/{name}", ignore_errors=True)
    os.makedirs(f"build/{name}")

    # Create new Git repository and fetch specified commit
    os.chdir(f"build/{name}")
    subprocess.run(["git", "init", "--quiet"])
    subprocess.run(["git", "remote", "add", "origin", repo])
    subprocess.run(["git", "fetch", "--quiet", "origin", commit])
    subprocess.run(["git", "reset", "--quiet", "--hard", "FETCH_HEAD"])
    os.chdir("../..")
