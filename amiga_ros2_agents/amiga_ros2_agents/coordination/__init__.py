"""Where this robot's stack meets the fleet's.

Both agents here exist because ``amiga_ros2_coordinator`` speaks in the codec's
flat vocabulary and never parses XML. Something has to translate, and doing it
on this side of the boundary is what keeps that rule cheap.

``triage`` answers the two questions the coordinator cannot: has local recovery
run out (deterministic -- the planner and arbiter already decided), and what
should be done about the work that is left (a model, constrained to three typed
actions). ``mission_bridge`` answers the coordinator's questions *about the
plan* from a latched snapshot, so the port reads a variable instead of blocking
the coordinator's lock on a service call.

Read-only, both of them. Edits go to the arbiter in ``replanning``.
"""
