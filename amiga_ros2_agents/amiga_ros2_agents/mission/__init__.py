"""The mission document: its schema, and the tasks inside it.

``xsd`` resolves and loads ``amiga_btcpp.xsd`` -- the same schema ``bt_runner``
validates against, so a plan an agent accepts is a plan the executor will run.
``mission_tasks`` is the only module that turns behaviour-tree XML into task
records and back, which is what keeps the coordinator's rule (it never parses
XML) affordable.

Libraries, not agents: no rclpy here. The agents that *edit* the plan are in
``replanning``; the ones that report it to the fleet are in ``coordination``.
"""
