"""An episode's task description: its native task and seed, the instructions upstream's evaluator gives its policy,
as the backend states them after its reset (its `description`), and the episode the run initialized (`episode`).

The world node states it in the run log as one line when the episode starts, and `kinex-sim describe` reads that
line back from the live run, so describing a run never opens its episode again. Everything but `task`, `seed` and
`episode` is instruction text for the agent; `episode` is the operator's: how the run was launched and what the
backend initialized, which a harness checks before it hands the agent its instructions.
"""
import json

PREFIX = "task description: "
# Keys of a task description that are not instruction text.
OWNER_KEYS = ("task", "seed", "episode")


def task_description(task, sim, episode):
    return {"task": task or sim.default_task, "seed": sim.seed, **sim.description, "episode": episode}


def episode_record(sim, *, mode, early_stop, configuration=None, scene=None, recording=None):
    """The episode a world node initialized: its execution mode, control period and step limit (None: no limit, as
    a realtime run has), the launch's options, and what the backend resolved at reset (its `initialization`, such
    as the kitchen a RoboCasa seed picked)."""
    return {"execution_mode": mode, "period_s": float(sim.period),
            "max_steps": int(sim.max_steps) if sim.max_steps else None,
            "early_stop": bool(early_stop), "configuration": configuration or None, "scene": dict(scene or {}),
            "initialization": dict(getattr(sim, "initialization", None) or {}), "recording": recording}


def instructions(description):
    """The instruction text of a task description: what to give the agent."""
    return {name: text for name, text in description.items() if name not in OWNER_KEYS}


def logged(description):
    """The run log's line: the prefix, then the description as one line of JSON."""
    return PREFIX + json.dumps(description, ensure_ascii=False)


def stated(log):
    """The task description a run log states, or None before its episode has started."""
    with open(log, encoding="utf-8", errors="replace") as stream:
        for line in stream:
            _, found, rest = line.partition(PREFIX)
            if found:
                return json.loads(rest)
    return None
