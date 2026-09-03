"""Mission model compiler.

``promela.py`` compiles a behaviour tree into a Promela model and the set of
propositions it establishes -- ``action_pool`` names, per the XSD's
``ActionGroup``, what the compiler knows how to represent. This module used
to feed a formal LTL check (a specification generated from the mission text,
verified against this model with SPIN); that check has been removed. What's
left is used directly by tests: ``test_ontology.py`` checks the ontology
table against ``action_pool`` to catch schema drift, and
``test_task_synthesis.py`` compiles a synthesized task to check it is
structurally sound.

No rclpy here, no LLM, no network -- mechanical and testable on its own.
"""
