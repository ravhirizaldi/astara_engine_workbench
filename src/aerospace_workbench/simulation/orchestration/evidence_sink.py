"""Single writer for runtime mission events."""

from __future__ import annotations

import json
from typing import Any


class EvidenceSink:
    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []
        self._sequence = 0

    def record(
        self,
        time_s: float,
        body: str,
        name: str,
        *,
        source: str,
        detected_time_s: float | None = None,
        detail: Any = "",
    ) -> dict[str, Any]:
        row = {
            "time_s": float(time_s),
            "detected_time_s": float(
                time_s if detected_time_s is None else detected_time_s
            ),
            "sequence": self._sequence,
            "body": body,
            "event": name,
            "source": source,
            "detail": (
                detail
                if isinstance(detail, str)
                else json.dumps(detail, sort_keys=True)
            ),
        }
        self._sequence += 1
        self.events.append(row)
        return row
