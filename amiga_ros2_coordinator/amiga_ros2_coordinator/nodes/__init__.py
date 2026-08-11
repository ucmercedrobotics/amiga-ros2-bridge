"""The three processes this package installs.

    coordinator_node.py  ``coordinator``      -- ports declining every question
    sim_node.py          ``coordinator_sim``  -- the same node, ports connected
    escalate.py          ``escalate``         -- fire one escalation by hand

The pair at the top is the port design's own test. ``coordinator``'s nav and
mission ports refuse everything on purpose -- a robot that bids because nothing
answered takes work it cannot do, so refusing is the right default -- and
``coordinator_sim`` is the identical node with three objects passed to the
constructor and nothing else changed.

``escalate`` publishes what the triage agent publishes when the planner and the
arbiter have both given up, so a fleet can be made to do something without
waiting for a behaviour tree to genuinely fail. That makes it the injection
point for any bench or simulation run, not only the scripted demo.
"""
