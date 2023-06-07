# flake8: noqa F401
from .core import *


def check_gurobi_license():
    """
    Check if a Gurobi license is installed on the system.
    """
    from pathlib import Path
    import os

    gurobi_lic = os.environ.get(
        "GRB_LICENSE_FILE", os.path.join(Path.home(), "gurobi.lic")
    )
    if not os.path.exists(gurobi_lic):
        raise RuntimeError(
            f"No Gurobi license found!"
            " Please install a license first. Looked in '{gurobi_lic}'."
        )


check_gurobi_license()
