"""Where an early-stop ABC run ends, without MuJoCo: the evaluator's task_eval in (eval_policy.py: rollout_over)."""
import pytest

pytest.importorskip("mujoco")
from kinex_sim_abc.backend import rollout_stop  # noqa: E402


@pytest.mark.parametrize("task_eval, expected", [
    ({"success": False, "ever_success": False}, None),
    ({"success": True, "ever_success": True}, ("success", None)),
    # ball_tray_balancing succeeds from reset and ends only at its failure.
    ({"success": True, "ever_success": True, "ever_failed": False}, None),
    ({"success": False, "ever_success": False, "ever_failed": True, "ball_dropped": True, "inside_tray": False},
     ("failure", "ball_dropped, ball_left_tray")),
    # A count task reports ever_failed as always false, so it never ends early.
    ({"success": True, "ever_success": True, "ever_failed": False}, None),
])
def test_an_early_stop_run_ends_where_the_evaluator_ends_its_rollout(task_eval, expected):
    assert rollout_stop(task_eval) == expected
