"""What this layer is allowed to talk about: nouns, actions, capabilities.

    model.py         the nouns -- tasks, places, peers, fitness
    schema.py        the closed unions an interpretation may return
    capabilities.py  what this robot can do, read off the mission schema

Deliberately the bottom of the package: nothing here imports anything else in
it, so a value object cannot acquire an opinion about coordination. ``Target``
and ``Capability`` come from the codec rather than being restated, because both
are statements about the behaviour tree's schema and a second copy is how the
two would come to disagree about what a task is.

``schema.py`` is the reason a language model can sit in this system without the
state machine growing an open-ended interface: reasoning returns one of three
typed actions and the coordinator refuses anything else, so every path below it
stays predictable from its inputs.
"""
