#!/usr/bin/env python3

import json
import multiprocessing as mp
import os


def run_json(name):
    with open(name) as file:
        input = file.read()
    output = json.dumps(json.loads(input), sort_keys=True, indent=4) + "\n"
    if input != output:
        with open(name, "w") as file:
            file.write(output)


files = [
    os.path.join(dp, f)
    for dp, dn, fn in os.walk("./deps")
    for f in fn
    if f.endswith(".json")
]

with mp.Pool(mp.cpu_count()) as pool:
    pool.map(run_json, files)
