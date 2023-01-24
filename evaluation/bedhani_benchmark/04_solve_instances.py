from aemeasure import MeasurementSeries, read_as_pandas_table, Database
import slurminade
from cetsp_bnb2 import (
    Circle,
    Instance,
    compute_tour_by_2opt,
    branch_and_bound,
    Point,
    plot_solution,
    compute_tour_from_sequence,
)
import os
from pathlib import Path
import socket

# your supervisor will tell you the necessary configuration.
slurminade.update_default_configuration(
    partition="alg",
    constraint="alggen03",
    mail_user="krupke@ibr.cs.tu-bs.de",
    mail_type="ALL",
)
slurminade.set_dispatch_limit(200)

# Parameter
timelimit = 300
result_folder = "./results_04"
instances_path = "./instance_db"


def load_instances():
    db_ = Database(instances_path)
    data = db_.load()
    instances = {instance["instance"]: instance for instance in data}
    return instances

@slurminade.slurmify()
def run_for_instance(instance_name, timelimit):
    instances = load_instances()

    # This configuration allows you to change the basic behaviour of the
    # branch and bound algorithm. You can do further improvements, such
    # as improving the lower bound or adding solutions using the callback.

    configurations = [
        {
            "root": "ConvexHull",
            "branching": "ChFarthestCircleSimplifying",
            "search": "DfsBfs",
            "rules": ["GlobalConvexHullRule"],
            "num_threads": 8,
        },
        {
            "root": "ConvexHull",
            "branching": "ChFarthestCircleSimplifying",
            "search": "DfsBfs",
            "rules": ["LayeredConvexHullRule"],
            "num_threads": 8,
        },
    ]
    # This callback currently does nothing. However, you can use the context
    # object as described in pysrc/cetsp_bnb2/core/_cetsp_bindings.cpp
    # to access the current node, trajectory, sequence, solutions, etc.
    callback = lambda context: None

    instances0 = []
    for radius in [0.25, 0.5]:
        instance = Instance(
            [
                Circle(
                    Point(float(d["x"]), float(d["y"])),
                    0.0 if i == 0 else radius,
                )
                for i, d in enumerate(instances[instance_name]["circles"])
            ]
        )
        initial_solution = compute_tour_by_2opt(instance)
        instances0.append((radius, instance, initial_solution))

    with MeasurementSeries(result_folder) as ms:
        for configuration in configurations:
            for radius, instance, initial_solution in instances0:
                with ms.measurement() as m:
                    print(instance_name, radius)
                    ub, lb, stats = branch_and_bound(
                        instance, callback, initial_solution, timelimit, **configuration
                    )
                    m["configuration"] = configuration
                    m["instance"] = instance_name
                    m["ub"] = ub.get_trajectory().length()
                    m["lb"] = lb
                    m["radius"] = radius
                    m["n"] = len(instance)
                    m["timelimit"] = timelimit
                    m["stats"] = stats
                    m.save_metadata()
                    m.save_seconds()
                    print(ub.get_trajectory().length(), lb)


if __name__ == "__main__":
    # Read data
    instances = load_instances()
    # run_for_instance.distribute("bedhani/CETSP-25-10", timelimit)
    with slurminade.Batch(20) as batch:
        for instance in instances.keys():
            run_for_instance.distribute(instance, timelimit)
