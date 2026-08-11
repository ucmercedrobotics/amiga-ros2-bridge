"""How an agent is wired, with nothing about what it decides.

Four things every agent in this package needs and none of them mission-specific:
reaching a model (``llm``), rendering a prompt (``prompts``), announcing itself
(``status``) and being spun up (``spin``). Each is the *only* entry point to its
concern, which is the reason they live together -- there is one place to change
the model endpoint, one place that knows where templates are, one status
contract, one node lifecycle.

Nothing here imports from the sibling packages. A module that needs to know what
a mission or a task is belongs in ``mission``, not in this directory.
"""
