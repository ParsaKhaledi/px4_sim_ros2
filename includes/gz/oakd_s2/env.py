"""Read process environment values. Empty text means unset."""

from __future__ import annotations

from collections.abc import Mapping


def env_str(env: Mapping[str, str | None], name: str, default: str | None = None) -> str | None:
    """String from ``env``. Empty or missing uses ``default``.

    Whitespace-only text is empty. A present value is returned unchanged,
    not stripped.
    """
    raw = env.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return str(raw)


def env_int(env: Mapping[str, str | None], name: str, default: int) -> int:
    """Positive int from ``env``. Empty uses ``default``. ``"320.7"`` becomes 320.

    Fractional text is truncated with ``int(float(raw))``, not rejected.
    ``"320.7"`` is 320. That is the historical ``_env_int`` behavior.
    """
    raw = env.get(name)
    if raw is None or str(raw).strip() == "":
        return int(default)
    value = int(float(raw))
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {raw}")
    return value


def env_float(env: Mapping[str, str | None], name: str, default: float) -> float:
    """Float from ``env``. Empty or missing uses ``default``."""
    raw = env.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return float(raw)
