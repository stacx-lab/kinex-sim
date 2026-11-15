"""The constraint an early-stop RoboDojo run fails on, without Isaac: its reward manager's queries in."""
from types import SimpleNamespace

from rosbridge_robodojo.backend import violations

LIFT = ("is_lift", {"label": "t", "z_threshold": 0.01})
GRASP = ("is_grasp", {"label": "cup"})


def test_a_failure_names_the_queries_that_held_more_often_than_allowed():
    manager = SimpleNamespace(query_list=[[[[LIFT], 0, 1], [[GRASP], 2, 2]]],
                              trigger_query_list=[[([GRASP], [[LIFT, GRASP]], "level", False, 1, 3)]])
    assert violations(manager) == ("is_lift(label=t, z_threshold=0.01) held on 1 step, at most 0; "
                                   "is_lift(label=t, z_threshold=0.01) or is_grasp(label=cup) held on 3 steps, at most 1")
    assert violations(SimpleNamespace(query_list=[[[[LIFT], 0, 0]]], trigger_query_list=[[]])) == ""
