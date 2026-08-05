# Failure scenarios

Running a real mission and making one step of it fail, so the chain below the
behaviour tree has something to react to.

```bash
ros2 launch amiga_ros2_behavior_tree failure_scenario.launch.py \
    mission:=sample_leafs.xml fail_goals:="[60]" failure_mode:=nav_failed
```

The automated version is
[`amiga_ros2_agents/test/test_scenario_bt_fault.py`](../../../amiga_ros2_agents/test/test_scenario_bt_fault.py),
which runs the same launch file and follows the fault through to a task the
coordinator could announce.

## What is being tested, and what is not

This is the part worth being blunt about, because the distinction decides what
any result from it is worth.

**The wiring is tested here.** Does a navigation abort become a
`/bt/status_change` event? Does the event name the leaf that failed rather than
every ancestor of it? Does that leaf resolve to a *unit* of work — the move to
the tree and the sample together — with the right action set and the right
place? Does the unit survive being grafted into another robot's plan and still
validate against the schema? Those are mechanical questions with mechanical
answers, and injection is sound evidence for all of them.

**Reasoning quality is not tested here, and cannot be.** The triage agent reads
`/rosout`. An injected failure has the right words and no substance behind it:
there is no blocked row, no lost GPS fix, no tree that has actually been felled.
A model asked to judge whether a fault is permanent is being asked a question
the scenario has no real answer to, so whatever it says measures the harness.

That means the honest claim about these scenarios is narrow: *the pipeline
carries a fault from the tree to an auction and back to a plan edit.* Any claim
about how well the pipeline **decides** needs failures with something behind
them — Gazebo, or hardware.

## How the injection stays out of the evidence

Given the above, the failures still have to be indistinguishable from real ones
in everything the model can see, or the wiring test itself becomes
unrepresentative:

- The log lines are copied **verbatim** from the real nodes —
  `amiga-ros2-nav/.../waypoint_follower.py` and
  `ros2-kortex-control/.../pistachio_leaf_segmentation.py` — at the same
  severity, followed by the same `goal_handle.abort()`.
- No `inject`, `mock`, `simulate`, `fake` or `test` appears anywhere on a
  failing path.
- The mocks run under the **real nodes' names**, because the `[node]` prefix is
  part of the log line the triage agent reads.
- The failure is chosen by a launch **parameter**, never a topic. A
  `/mock/inject_fault` message would land in the same 30-second `/rosout` window
  the agent reads, which is the whole problem again.
- Failures happen at the point in the goal's life where the real ones do —
  after feedback has been published, not on receipt — so the timing does not
  give it away either.

See [`failure_modes.hpp`](../../include/amiga_ros2_behavior_tree/mocks/failure_modes.hpp).

## Failure modes

Each is a real failure path in the node it stands in for. They are **not**
interchangeable, and picking the wrong one asks the pipeline a different
question:

| mode | line | permanent? |
| --- | --- | --- |
| `nav_failed` | `NavigateViaLidar action failed` | no — a peer might get there |
| `tree_info_unavailable` | `Orchard GetTreeInfo service unavailable` | no — the service comes back |
| `tree_info_empty` | `GetTreeInfo returned empty result` | **yes** — the tree does not exist |
| `no_waypoint` | `No valid target waypoint available from orchard data` | **yes** — nowhere to approach from |
| `no_point_cloud` | `No point cloud available.` | no — this robot's camera |
| `no_leaves` | `No leaves detected in the point cloud.` | **yes** — nothing there to sample |

The permanent/transient split is the judgement the whole `re_delegate` vs
`drop_task` decision turns on, which makes these six the natural test matrix for
the reasoning step — *once it is being run against failures that are real*.

## Arguments

| argument | default | meaning |
| --- | --- | --- |
| `mission` | `sample_leafs.xml` | An example by filename, or an absolute path |
| `fail_goals` | `[60]` | Tree ids whose navigation fails; `[0]` for the arm |
| `fail_after_n` | `0` | Attempts at each goal that succeed first |
| `failure_mode` | `nav_failed` | One of the six above |
| `xml_validation` | `true` | Left on: a mission the robot would refuse is not a scenario |
| `agents` | `false` | Also start planner, arbiter and triage (needs a model endpoint) |

`fail_after_n` counts attempts **per goal**, which is what makes transient
faults expressible: `fail_after_n:=2` under a `RetryUntilSuccessful num_attempts="3"`
is a fault the tree recovers from by itself, and it should never reach the
radio. That negative case is as much a property of the design as the positive
one — coordination is for what local recovery could not fix.

## A note on the missions

The scenarios run against the files in `examples/`, not against fixtures. A
fixture written for a test is a place to encode the same assumption twice and
prove nothing, and this has already earned its keep: the flat structure of
`sample_leafs.xml` — one `Sequence` holding all six leaves — is what exposed a
lookup that resolved *every* failing node to the mission's first unit. A nested
fixture had passed.

`relative_move.xml` does not currently validate against `amiga_btcpp.xsd`: it
uses `action_name="navigate_to_pose_in_frame"` where the schema fixes
`move_in_frame` and `rotate_in_frame`, so `bt_runner` refuses to build a tree
from it. It is skipped rather than worked around, and pinned by
`test_which_example_missions_the_schema_accepts`.
