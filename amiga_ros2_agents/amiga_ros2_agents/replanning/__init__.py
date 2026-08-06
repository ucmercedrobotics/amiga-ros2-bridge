"""The self-correction loop: the agents that repair a plan in flight.

    /bt/status_change -> mission_planner -> /mission/candidate_xml -> arbiter
                                                                        |
                                              /mission/xml <------------+

The planner proposes and the arbiter disposes. That split is deliberate: the
node that writes the plan is not the node that judges it, so a model cannot
approve its own work. **The arbiter is the sole writer of** ``/mission/xml``,
including for edits that originate off-robot -- a task won at auction arrives
over ``VerifyReplan`` and takes the same path through the same checks.

``world_state`` sits here because the planner's context window is what it feeds;
it aggregates action feedback into the telemetry a replan reads instead of
blocking on a request.
"""
