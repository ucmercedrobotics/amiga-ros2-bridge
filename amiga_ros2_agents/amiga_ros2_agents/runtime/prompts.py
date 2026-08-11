"""
prompts.py

Single Jinja2 entry point shared by every agent in this package.

Prompts live as `.j2` templates under `prompts/<agent>/`, installed into the
package share directory. Agents call render() and never build prompt strings by
hand — that keeps prompt tuning out of the Python and lets each agent inject its
own runtime context (AP vocabularies, world state, failure events).

Env vars:
    AGENT_PROMPT_DIR    override the template root; point it at the source tree
                        (`<repo>/amiga_ros2_agents/prompts`) to iterate on
                        prompts without rebuilding.
"""

import os

from ament_index_python.packages import get_package_share_directory
from jinja2 import Environment, FileSystemLoader, StrictUndefined

PROMPT_ROOT = os.environ.get("AGENT_PROMPT_DIR") or os.path.join(
    get_package_share_directory("amiga_ros2_agents"), "prompts"
)

_env = Environment(
    loader=FileSystemLoader(PROMPT_ROOT),
    # A mistyped variable must fail loudly. The default (Undefined) renders it as
    # an empty string, which silently drops part of a system prompt.
    undefined=StrictUndefined,
    trim_blocks=True,
    lstrip_blocks=True,
)


def render(name: str, **variables) -> str:
    """Render a template by path relative to the prompt root.

    >>> render("ltl_gen/system.j2", ap_vocabulary=["at_tree_1"])
    """
    return _env.get_template(name).render(**variables).strip()
