"""Where the ports meet the real robot. Every ROS dependency lives here.

    nav_ports.py         NavInterface, over the GPS fix and the orchard model
    mission_ports.py     MissionInterface, over the mission bridge's snapshot
    triage_client.py     interpret_anomaly, over the triage agent's service
    replanner_client.py  the arbiter, over VerifyReplan

They exist as separate files from ``engine`` for one reason: the state machine
has to stay testable without a middleware, and anything that can block has to
stay out of it.

**Two constraints run through all four.** Nothing here may block the
coordinator's lock -- ``_assess`` calls nav and mission from ``_on_announce``,
under the lock ``tick`` and ``on_message`` need, so a service round trip there
stops this robot's heartbeats and retransmit deadlines and the fleet reads that
as a robot that died. And nothing here may parse mission XML: what crosses is
the flat vocabulary the codec defines, which is what keeps that rule cheap.

The two clients answer slow questions -- a language model, a model checker --
so both dispatch off-thread and route a late answer back through the anomaly
path rather than making the caller wait.
"""
