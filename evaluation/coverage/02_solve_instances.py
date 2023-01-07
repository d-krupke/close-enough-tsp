from aemeasure import MeasurementSeries, read_as_pandas_table, Database
import slurminade
from cetsp_bnb2 import Circle, Instance, compute_tour_by_2opt, branch_and_bound, Point, plot_solution, compute_tour_from_sequence

# your supervisor will tell you the necessary configuration.
slurminade.update_default_configuration(partition="alg", constraint="alggen03")
slurminade.set_dispatch_limit(200)

# Parameter
timelimit = 300
result_folder = "./results"
instances_path = "./instance_db"


def load_instances():
    db_ = Database(instances_path)
    data = db_.load()
    instances = {instance["instance"]: instance  for instance in data}
    return instances


@slurminade.slurmify()
def run_for_instance(instance_name, timelimit):
    instances = load_instances()
    instance = Instance([Circle(Point(float(d["x"]), float(d["y"])), float(d["radius"])) for d in instances[instance_name]["circles"]])
    with MeasurementSeries(result_folder) as ms:
        with ms.measurement() as m:
            initial_solution = compute_tour_by_2opt(instance)
            ub, lb = branch_and_bound(instance, lambda event: None, initial_solution, timelimit)
            m["instance"]=instance_name
            m["ub"]=ub.length()
            m["lb"]=lb
            #m["n"] = len(instance)
            m["timelimit"]=timelimit
            m.save_metadata()
            m.save_seconds()

if __name__ == "__main__":
    # Read data
    instances = load_instances()
    for instance in instances.keys():
        run_for_instance.distribute(instance, timelimit)