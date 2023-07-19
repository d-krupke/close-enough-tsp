"""
This script runs the benchmark.
It is set up for distributed execution using Slurm.
If your system does not support Slurm, it will run the benchmark locally,
and ignore the Slurm configuration.

You can rerun the benchmark at any time, and it will only run the instances
and configurations that have not been completed yet. This way,
you can iteratively improve your algorithm and rerun the benchmark.
"""

import json

import pandas as pd
import slurminade

import os
from pathlib import Path
import socket
from algbench import Benchmark

# hack for gurobi licence on alg workstations. TODO: Find a nicer way
os.environ["GRB_LICENSE_FILE"] = os.path.join(
    Path.home(), ".gurobi", socket.gethostname(), "gurobi.lic"
)
from cetsp_bnb2 import (
    Circle,
    Instance,
    optimize,
    Point,
)

# your supervisor will tell you the necessary configuration.
slurminade.update_default_configuration(
    partition="alg",
    constraint="alggen03",
    mail_user="krupke@ibr.cs.tu-bs.de",
    mail_type="ALL",
)
slurminade.set_dispatch_limit(100)

# Parameter
timelimit = 120

benchmark = Benchmark("results")
import gzip
import json

with open("00_instances/benchmark.json.gz", "rb") as f:
    instances = json.loads(gzip.decompress(f.read()).decode("utf-8"))


def measure(instance_name, configuration, _instance):  # noqa: ARG001
    """
    This function is called by the benchmark for each instance and configuration.
    """
    ub, lb, stats = optimize(
        _instance, **configuration
    )
    return {
        "ub": ub.get_trajectory().length(),
        "lb": lb,
        "stats": stats,
        "n": len(_instance),
    }


@slurminade.slurmify()
def run_for_instance(instance_name):
    instance = Instance(
        [
            Circle(Point(float(d["x"]), float(d["y"])), float(d["radius"]))
            for d in instances[instance_name]["disks"]
        ]
    )
    # This configuration allows you to change the basic behaviour of the
    # branch and bound algorithm. You can do further improvements, such
    # as improving the lower bound or adding solutions using the callback.
    with open("./configurations.json") as f:
        configurations = json.load(f)

    for configuration in configurations:
        # run the benchmark for this instance and configuration
        # if the instance and configuration are still missing
        # from the benchmark. Otherwise, the call returns immediately.
        benchmark.add(
            measure,
            instance_name=instance_name,
            configuration=configuration,
            _instance=instance,
        )


@slurminade.slurmify()
def commpress():
    benchmark.compress()


if __name__ == "__main__":
    with slurminade.Batch(10) as batch:
        for instance_name in instances:
            run_for_instance.distribute(instance_name)
        commpress.wait_for(batch.flush()).distribute()
