"""The pre-registration, held to its own rules.

registry.py is what makes this ablation auditable rather than merely
plausible: ground truth committed before a trial runs, in one file, read by
both harnesses and the analyzer alike. That claim is worth nothing if the
file itself is malformed -- a scenario with no scorer, an `expected` that is
not one of the actions a policy could actually return, or worst of all, a
fleet scenario that leaks its distinguishing fact into a typed field and
quietly answers the question the study exists to ask. Every test below checks
one way this file could undermine its own premise, mechanically, rather than
by a human re-reading six dictionaries and trusting themselves.

Runs on bare Python. registry.py imports nothing from amiga_ros2_comms or
amiga_ros2_coordinator -- see its module docstring for why -- so nothing here
needs a container, a colcon build, or rclpy on the path.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import registry  # noqa: E402


def test_every_single_robot_scenario_has_a_pre_registered_expected_string():
    for scenario_id, scenario in registry.SINGLE_SCENARIOS.items():
        assert (
            isinstance(scenario.get("expected"), str) and scenario["expected"]
        ), f"{scenario_id} has no pre-registered `expected` hypothesis"


def test_every_single_robot_scorer_is_callable_and_returns_a_bool():
    for scenario_id, scenario in registry.SINGLE_SCENARIOS.items():
        scorer = scenario["scorer"]
        assert callable(scorer), f"{scenario_id}'s scorer is not callable"
        for decision in registry.SINGLE_DECISIONS:
            result = scorer(decision)
            assert isinstance(result, bool), (
                f"{scenario_id}'s scorer returned {result!r} for decision "
                f"{decision!r}, not a bool"
            )


def test_stuck_robot_scores_only_abort_as_correct():
    # Pinned individually, not just "is callable": this is the scenario
    # whose whole point is recognizing futility, and a scorer that drifted
    # to accepting "accept" would silently erase the one failure mode this
    # scenario exists to catch.
    scorer = registry.SINGLE_SCENARIOS["stuck_robot"]["scorer"]
    assert scorer("abort") is True
    assert scorer("accept") is False


def test_missing_tree_scores_only_accept_as_correct():
    scorer = registry.SINGLE_SCENARIOS["missing_tree"]["scorer"]
    assert scorer("accept") is True
    assert scorer("abort") is False


def test_every_fleet_scenario_expected_action_is_inside_the_closed_schema():
    for scenario_id, scenario in registry.FLEET_SCENARIOS.items():
        assert scenario["expected"] in registry.FLEET_ACTIONS, (
            f"{scenario_id}'s expected action {scenario['expected']!r} is not "
            f"one of {sorted(registry.FLEET_ACTIONS)}"
        )


def test_every_fleet_scorer_is_callable_and_agrees_with_its_own_expected():
    for scenario_id, scenario in registry.FLEET_SCENARIOS.items():
        scorer = scenario["scorer"]
        assert callable(scorer), f"{scenario_id}'s scorer is not callable"
        assert (
            scorer(scenario["expected"]) is True
        ), f"{scenario_id}'s scorer does not call its own `expected` correct"
        for other in registry.FLEET_ACTIONS - {scenario["expected"]}:
            assert scorer(other) is False, (
                f"{scenario_id}'s scorer calls {other!r} correct too, "
                "alongside its own expected action"
            )


def test_every_fleet_scenario_names_a_shedder_and_two_peers():
    for scenario_id, scenario in registry.FLEET_SCENARIOS.items():
        node_ids = [robot["node_id"] for robot in scenario["robots"]]
        assert (
            node_ids.count(scenario["shedder_id"]) == 1
        ), f"{scenario_id} does not name exactly one shedder among its robots"
        assert len(scenario["robots"]) == 3, (
            f"{scenario_id} has {len(scenario['robots'])} robots, expected 3 "
            "(one shedder, two peers)"
        )


def test_the_baseline_correct_scenarios_have_the_ground_truth_the_spec_pins():
    # From the spec's table: structured state alone is sufficient for these
    # three, so the deterministic arm is expected to get them right.
    assert registry.FLEET_SCENARIOS["peer_capable"]["expected"] == "re_delegate"
    assert registry.FLEET_SCENARIOS["no_peers"]["expected"] == "drop_task(hold)"
    assert registry.FLEET_SCENARIOS["no_capability"]["expected"] == "drop_task(hold)"


def test_the_baseline_wrong_scenarios_have_the_ground_truth_the_spec_pins():
    assert registry.FLEET_SCENARIOS["transient"]["expected"] == "drop_task(hold)"
    assert registry.FLEET_SCENARIOS["fleet_wide"]["expected"] == "drop_task(drop)"
    assert registry.FLEET_SCENARIOS["new_work"]["expected"] == "add_task"


def test_text_only_variants_share_peer_capables_robots_object_identity():
    # Identity, not merely equality: peer_capable, transient, fleet_wide and
    # new_work must be the *same* fleet composition, so an edit to one cannot
    # silently drift the other three out from under it and leave the study
    # comparing scenarios that were never structurally identical.
    reference = registry.FLEET_SCENARIOS["peer_capable"]["robots"]
    for scenario_id in registry.TEXT_ONLY_VARIANTS:
        assert (
            registry.FLEET_SCENARIOS[scenario_id]["robots"] is reference
        ), f"{scenario_id}'s robots is not peer_capable's own object"


def test_text_only_variants_share_peer_capables_task_capabilities():
    reference = registry.FLEET_SCENARIOS["peer_capable"]["task_capabilities"]
    for scenario_id in registry.TEXT_ONLY_VARIANTS:
        assert (
            registry.FLEET_SCENARIOS[scenario_id]["task_capabilities"] == reference
        ), f"{scenario_id}'s task_capabilities differs from peer_capable's"


def test_text_only_variants_differ_from_peer_capable_only_in_reason():
    # The mechanical form of the study's central asymmetry: walk every key a
    # fleet scenario dict carries and confirm the only one allowed to differ
    # between peer_capable and each of its three text-only variants is
    # `reason`. `expected` and `scorer` are permitted to differ too -- they
    # are what the scenario is *for*, not structural state a policy reads --
    # everything else (robots, shedder_id, task_capabilities) must not.
    baseline = registry.FLEET_SCENARIOS["peer_capable"]
    structural_keys = {"robots", "shedder_id", "task_capabilities"}
    for scenario_id in registry.TEXT_ONLY_VARIANTS:
        variant = registry.FLEET_SCENARIOS[scenario_id]
        for key in structural_keys:
            assert variant[key] == baseline[key], (
                f"{scenario_id} differs from peer_capable in structural field "
                f"{key!r}; distinguishing information must live only in "
                "`reason`"
            )
        assert variant["reason"] != baseline["reason"], (
            f"{scenario_id} has the same `reason` as peer_capable, so nothing "
            "distinguishes it at all"
        )


def test_text_only_variant_reasons_carry_no_typed_flag_by_naming_convention():
    # Not a proof (a scorer cannot see inside a string's meaning) but a
    # tripwire: if a future edit adds a structured-sounding key to a fault
    # dict (`fault_is_fleet_wide`, `transient: true`) rather than writing
    # english into `reason`, this catches the shape of that mistake -- a
    # fleet scenario dict with a key this module never defined.
    known_keys = {
        "robots",
        "shedder_id",
        "task_capabilities",
        "reason",
        "expected",
        "scorer",
    }
    for scenario_id, scenario in registry.FLEET_SCENARIOS.items():
        extra = set(scenario.keys()) - known_keys
        assert not extra, (
            f"{scenario_id} carries undocumented keys {sorted(extra)} -- a "
            "typed field smuggling in what `reason` is supposed to carry "
            "alone would look exactly like this"
        )


def test_git_sha_returns_a_string_and_does_not_raise():
    sha = registry.git_sha()
    assert isinstance(sha, str) and sha
