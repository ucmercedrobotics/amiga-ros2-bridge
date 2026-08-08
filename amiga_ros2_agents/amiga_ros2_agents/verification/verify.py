"""Run SPIN over a model and a formula. The one place that shells out.

``promela.py`` builds the model, ``ltl_gen`` supplies the formula, this decides
whether the first satisfies the second.

**The verdict comes from ``errors: N``, never from the exit status.** ``spin
-search`` exits 0 whether it proved the property or refuted it; only a parse or
toolchain failure is non-zero. Reading the exit code as the verdict -- which the
upstream planner's ``_ltl_validation`` does, recovering later by checking
whether a ``.trail`` file appeared -- scores a genuine violation as a pass.
Since that is the one mistake that makes the entire verification step
worthless, everything here is arranged so a result must be *positively*
established:

    rc != 0                     -> failed, toolchain or syntax
    no "errors:" line at all    -> failed, output not understood
    errors: 0                   -> holds
    errors: N > 0               -> violated, fetch the counterexample

The absent-and-ambiguous cases fail closed. A verifier that did not answer has
not verified anything.

Each run gets its own directory. SPIN writes ``pan``, ``pan.c`` and the
``.trail`` next to the model, so a shared working directory makes concurrent or
successive runs read each other's leftovers -- the upstream one is a
``NamedTemporaryFile`` in the process cwd and races across retries.

No ROS, no LLM, no network. Needs ``spin`` on PATH; the Dockerfile builds it.
"""

import os
import re
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from typing import Optional

#: SPIN's summary line, e.g. "State-vector 28 byte, depth reached 13, errors: 1"
_ERRORS_RE = re.compile(r"\berrors:\s*(\d+)")

#: Enough for any plan a robot carries. A model that needs longer has something
#: wrong with it, and hanging the arbiter is not a better answer than failing.
DEFAULT_TIMEOUT_SEC = 60.0

SPIN_BINARY = os.environ.get("SPIN_BINARY", "spin")


@dataclass(frozen=True)
class Verdict:
    """Held, violated, or could not be established -- three, not two.

    ``holds`` is only ever true when SPIN said so. ``error`` distinguishes "this
    plan is wrong" from "we do not know whether this plan is wrong", which the
    caller must not collapse: the first is a rejection with a reason, the second
    is a verifier outage.
    """

    holds: bool
    #: SPIN's guided replay of the violating run, when there is one.
    counterexample: str = ""
    #: Set when no verdict could be reached at all. Implies ``not holds``.
    error: Optional[str] = None

    @property
    def conclusive(self) -> bool:
        return self.error is None


def spin_available() -> bool:
    """Is there a checker to call? Lets callers degrade loudly, not silently."""
    return shutil.which(SPIN_BINARY) is not None


def check(
    promela_source: str,
    formula: str,
    timeout_sec: float = DEFAULT_TIMEOUT_SEC,
) -> Verdict:
    """Check ``promela_source`` against ``formula``.

    ``formula`` is a bare LTL expression in SPIN syntax -- no ``ltl name { }``
    wrapper, which is what ``ltl_gen`` already publishes.
    """
    formula = " ".join(formula.split()).strip()
    if not formula:
        return Verdict(False, error="empty formula")
    if not spin_available():
        return Verdict(False, error=f"{SPIN_BINARY} not found on PATH")

    source = f"{promela_source}\nltl mission {{ {formula} }}\n"

    with tempfile.TemporaryDirectory(prefix="ltlcheck-") as workdir:
        model = os.path.join(workdir, "mission.pml")
        with open(model, "w") as handle:
            handle.write(source)

        try:
            run = _spin(["-search", "-a", "-O2", "mission.pml"], workdir, timeout_sec)
        except subprocess.TimeoutExpired:
            return Verdict(False, error=f"spin timed out after {timeout_sec:.0f}s")
        except OSError as exc:
            return Verdict(False, error=f"could not run {SPIN_BINARY}: {exc}")

        output = run.stdout + run.stderr

        if run.returncode != 0:
            return Verdict(False, error=_first_error(output))

        found = _ERRORS_RE.search(output)
        if found is None:
            # Never seen in practice, and precisely why it is checked: without
            # this, an unparsed output would fall through to "holds".
            return Verdict(False, error=f"no verdict in spin output: {output[-400:]}")

        if int(found.group(1)) == 0:
            return Verdict(True)

        return Verdict(False, counterexample=_trail(workdir, timeout_sec))


def _spin(args, workdir: str, timeout_sec: float) -> subprocess.CompletedProcess:
    return subprocess.run(
        [SPIN_BINARY, *args],
        cwd=workdir,
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )


def _trail(workdir: str, timeout_sec: float) -> str:
    """Replay the counterexample SPIN just wrote.

    Returned verbatim. It names the propositions the never claim was waiting on
    and the variable values it stopped at, which is the most useful thing the
    planner can be handed when told to try again.
    """
    try:
        replay = _spin(["-t", "mission.pml"], workdir, timeout_sec)
    except (subprocess.TimeoutExpired, OSError) as exc:
        return f"(counterexample unavailable: {exc})"
    trace = (replay.stdout + replay.stderr).strip()
    return trace or "(spin reported a violation but produced no trail)"


def _first_error(output: str) -> str:
    """Pull the informative line out of a SPIN failure."""
    for line in output.splitlines():
        if "Error:" in line or line.startswith("spin:"):
            return line.strip()
    return (output.strip() or "spin failed with no output")[-400:]
