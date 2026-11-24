"""Where an early-stop HandsOn run ends, without Isaac: a native step's info and the evaluator's reason in."""
import pytest

from kinex_sim_handson.backend import failure_cause, native_stop


def returned(**info):
    return {"terminated": bool(info.get("succeed") or info.get("failed")), "truncated": False, "info": info}


@pytest.mark.parametrize("info, reason, timed_out, expected", [
    ({"succeed": False}, None, False, None),
    ({"succeed": True}, "native_terminated", False, ("success", None)),
    # A step that succeeds and fails at once is a success, as the evaluator scores info["succeed"].
    ({"succeed": True, "failed": True}, "native_terminated", False, ("success", None)),
    ({"succeed": False, "failed": True, "ycb01_failure_reason": "ball_or_hand_touched_gate"}, "native_terminated",
     False, ("failure", "native_failed: ycb01_failure_reason=ball_or_hand_touched_gate")),
    # t11's evaluator rule ends the trial before the native flags assert termination.
    ({"succeed": False}, "native_c12_direct_hand_contact", False, ("failure", "native_c12_direct_hand_contact")),
    # t17 can end its trial with neither flag; a core environment's time limit is no failure.
    ({"succeed": False}, "native_terminated", False, ("failure", "native_terminated")),
    ({"succeed": False}, "native_terminated", True, None),
    ({"succeed": False}, "native_truncated", True, None),
])
def test_an_early_stop_run_ends_where_the_evaluator_ends_the_trial(info, reason, timed_out, expected):
    assert native_stop(returned(**info), reason, timed_out) == expected


def test_a_failure_cause_names_the_native_entries_that_explain_it():
    info = {"succeed": False, "failed": True, "failure_reason": "door_speed_explosion", "failure_reasons": ["a"],
            "c05_failed_early_click": True, "b320_failure_reason": "", "manual_reset": False, "steps": 3}
    assert failure_cause("native_failed", info) == (
        "native_failed: failure_reason=door_speed_explosion, c05_failed_early_click=True")
    assert failure_cause("native_failed", {"failed": True}) == "native_failed"
