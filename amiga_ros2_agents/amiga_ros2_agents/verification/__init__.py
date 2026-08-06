"""Formal verification: the LTL specification, the model, and the checker.

The two halves are produced independently and that separation is the whole
point -- ``ltl`` sees the ``<Mission>`` text and never the tree, ``promela``
compiles the tree and never sees the formula. Neither can be bent to fit the
other, so their agreement is evidence rather than construction. They meet in
``ltl_gate``, which is the ordered set of checks the arbiter runs before any
plan is committed.

    ltl.py        mission text -> LTL formula        (the specification)
    promela.py    behaviour tree -> Promela model    (the implementation)
    verify.py     SPIN, and the three-way verdict
    ltl_gate.py   the checks, in order, cheapest first

``ltl_gen_node`` exposes the first of those over ROS for callers that want a
formula on its own, and is the only agent in this directory. The arbiter
deliberately does not go through it -- see that module's docstring.

Of the four libraries only ``ltl`` reaches a model, and none of them import
rclpy. The rest is mechanical and testable with no ROS, no network and no LLM,
which is the reason ``test_verify.py`` can run SPIN for real instead of mocking
the one component whose behaviour is the entire claim.
"""
