"""
llm.py

Single LiteLLM entry point shared by every agent in this package.

Model selection lives here and nowhere else — agents call complete() and never
touch litellm directly. LiteLLM is used so the same call works against a local
vLLM/ollama endpoint or a cloud provider just by changing env vars.

Env vars (AGENT_* preferred; the LOCAL_* names the run scripts already export
are honoured as a fallback):
    AGENT_MODEL / LOCAL_MODEL          e.g. hosted_vllm/openai/gpt-oss-20b, gpt-5.6-sol
    AGENT_API_BASE / LOCAL_API_BASE    "" = the provider's official endpoint
    AGENT_TEMPERATURE
    AGENT_MAX_TOKENS
    ENV_FILE_PATH                      .env holding provider API keys
"""

import os
import time
from typing import Optional

import litellm
from dotenv import load_dotenv

# Let LiteLLM silently drop params a given provider doesn't support, so the same
# call works across local and cloud models.
litellm.drop_params = True

ENV_FILE_PATH = os.environ.get("ENV_FILE_PATH", "/amiga-ros2-bridge/.env")
load_dotenv(ENV_FILE_PATH)


def _env(*names: str, default: str = "") -> str:
    """First non-empty value among `names`, else `default`."""
    for name in names:
        value = os.environ.get(name)
        if value:
            return value
    return default


MODEL = _env("AGENT_MODEL", "LOCAL_MODEL", default="hosted_vllm/openai/gpt-oss-20b")
API_BASE = _env("AGENT_API_BASE", "LOCAL_API_BASE", default="http://localhost:8000/v1")
TEMPERATURE = float(_env("AGENT_TEMPERATURE", default="0.2"))
MAX_TOKENS = int(_env("AGENT_MAX_TOKENS", default="2048"))

MAX_RATE_LIMIT_RETRIES = 3
RATE_LIMIT_BACKOFF_SEC = 1.0


def complete(
    system: str,
    user: str,
    *,
    model: Optional[str] = None,
    api_base: Optional[str] = None,
    temperature: Optional[float] = None,
    max_tokens: Optional[int] = None,
) -> str:
    """One system+user completion. Returns the assistant text (never None).

    Retries a bounded number of times on rate limits; any other provider error
    propagates to the caller.
    """
    messages = [
        {"role": "system", "content": system},
        {"role": "user", "content": user},
    ]
    base = API_BASE if api_base is None else api_base

    last_exc: Optional[Exception] = None
    for attempt in range(MAX_RATE_LIMIT_RETRIES):
        try:
            response = litellm.completion(
                model=model or MODEL,
                messages=messages,
                temperature=TEMPERATURE if temperature is None else temperature,
                max_tokens=MAX_TOKENS if max_tokens is None else max_tokens,
                api_base=base or None,
            )
            # Some providers return a None content on an empty/filtered reply
            return (response.choices[0].message.content or "").strip()
        except litellm.exceptions.RateLimitError as exc:
            last_exc = exc
            time.sleep(RATE_LIMIT_BACKOFF_SEC * (attempt + 1))

    raise RuntimeError(
        f"rate limited by {model or MODEL} after {MAX_RATE_LIMIT_RETRIES} attempts"
    ) from last_exc


def strip_code_fence(text: str) -> str:
    """Remove a leading/trailing markdown code fence if the model added one."""
    text = text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        lines = lines[1:]  # drop opening ``` or ```promela
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines).strip()
    return text
