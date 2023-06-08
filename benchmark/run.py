import json

import pandas as pd
import slurminade
from cetsp_bnb2 import (
    Circle,
    Instance,
    compute_tour_by_2opt,
    branch_and_bound,
    Point,
)
import os
from pathlib import Path
import socket
from algbench import Benchmark

# hack for gurobi licence on alg workstations. TODO: Find a nicer way
os.environ["GRB_LICENSE_FILE"] = os.path.join(
    Path.home(), ".gurobi", socket.gethostname(), "gurobi.lic"
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
instances = pd.read_json("./instances.json.zip").to_dict()["circles"]


def measure(instance_name, configuration, timelimit, _instance):
    initial_solution = compute_tour_by_2opt(_instance)
    ub, lb, stats = branch_and_bound(
        _instance, (lambda context: None), initial_solution, timelimit, **configuration
    )
    return {
        "ub": ub.get_trajectory().length(),
        "lb": lb,
        "stats": stats,
        "n": len(_instance),
    }


@slurminade.slurmify()
def run_for_instance(instance_name, timelimit):
    instance = Instance(
        [
            Circle(Point(float(d["x"]), float(d["y"])), float(d["radius"]))
            for d in instances[instance_name]
        ]
    )
    # This configuration allows you to change the basic behaviour of the
    # branch and bound algorithm. You can do further improvements, such
    # as improving the lower bound or adding solutions using the callback.
    with open("./configurations.json") as f:
        configurations = json.load(f)

    for configuration in configurations:
        benchmark.add(
            measure,
            instance_name=instance_name,
            configuration=configuration,
            timelimit=timelimit,
            _instance=instance,
        )


@slurminade.slurmify()
def commpress():
    benchmark.compress()


if __name__ == "__main__":
    with slurminade.Batch(20) as batch:
        for instance in instances.keys():
            run_for_instance.distribute(instance, timelimit)
        commpress.wait_for(batch.flush()).distribute()
