"""Source project discovery, independent of ROS and simulator installations."""
import os
from pathlib import Path


def project_root(value=None):
    explicit = value or os.environ.get("KINEX_SIM_PROJECT")
    candidates = ([Path(explicit)] if explicit else
                  [Path.cwd(), *Path.cwd().parents, Path(__file__).resolve().parents[2]])
    for candidate in candidates:
        root = candidate.expanduser().resolve()
        if (root / "src/kinex_sim").is_dir() and (root / "pixi.toml").is_file():
            return root
    raise ValueError("Select a kinex-sim checkout with --project or KINEX_SIM_PROJECT")
