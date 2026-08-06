"""The seams, declared. Implemented in ``adapters``, faked in ``test``.

    interfaces.py  nav, mission and preemption -- this robot's own nodes
    reasoning.py   the two questions that need judgement, and the stubs

Protocols, not base classes, and no ROS types in either file. That is what lets
``engine`` be written against the interface rather than the behaviour, and it
is why a whole auction with backoff windows and peer timeouts runs in
microseconds in the acceptance suite with nothing sleeping and no middleware.

Swapping a fake nav for the real Nav2 client must not touch one line of the
contract-net logic. If it does, the boundary was drawn in the wrong place.
"""
