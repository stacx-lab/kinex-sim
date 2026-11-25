"""Local run records and Linux process identity checks."""
import json
import os
from pathlib import Path
import time

SCHEMA = "kinex-sim/run/1"


def identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return {"pid": pid, "start": fields[19],
                "boot": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
    except (OSError, IndexError):
        return None


def alive(record):
    owner = record.get("owner")
    return bool(owner and identity(owner["pid"]) == owner)


def read(path):
    value = json.loads(path.read_text())
    if value.get("schema") != SCHEMA or value.get("uid") != os.getuid():
        raise ValueError("Not a kinex-sim run owned by this user")
    return value


def directory(root):
    return root / ".cache/kinex-sim/runs"


def own(root):
    """This user's run records by path. What else lies in the directory, such as a record another schema or user
    left, is not a run to select."""
    for path in directory(root).glob("*/run.json"):
        try:
            yield path, read(path)
        except ValueError:
            continue


def select(root, run=None):
    if run:
        candidate = Path(run).expanduser()
        if candidate.is_dir():
            candidate /= "run.json"
        if not candidate.is_file():
            if Path(run).name != run:
                raise ValueError("Select a run ID or existing run.json")
            candidate = directory(root) / run / "run.json"
        if not candidate.is_file():
            raise ValueError(f"Unknown run: {run}")
        return candidate, read(candidate)
    matches = [(path, record) for path, record in own(root)
               if alive(record) and record["state"] in ("starting", "ready", "stopping")]
    if len(matches) != 1:
        raise ValueError(f"Found {len(matches)} live runs; select an explicit RUN_ID")
    return matches[0]


def on_domain(root, domain):
    """The live run on a ROS domain; each live run leases its own."""
    for path, record in own(root):
        if alive(record) and record.get("domain_id") == domain:
            return path, record
    raise ValueError(f"No live run on ROS domain {domain}; launch one first")


def wait_stopped(path, timeout=45.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        record = read(path)
        if not alive(record):
            return record
        time.sleep(.05)
    raise TimeoutError("Run did not stop; inspect its log and process identity")
