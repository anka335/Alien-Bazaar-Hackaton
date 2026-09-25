"""Run log: every decision observation and what the state machine did with it.

`<runs_dir>/<run_id>/`:
- `run.json`: config at start; counters and end reason once the run ends;
- `<cycle:04d>_<phase>.npz` + `.png`: the observation (`sorter.core.io`);
- `<cycle:04d>_<phase>.json`: the vision result (without arrays) and the decision.

A phase repeated within one cycle gets a suffix (`0003_sense_box_2`). A re-sense of the same
observation writes only a new .json that points at the existing .npz.
Write errors are logged and never stop the loop.
"""

from __future__ import annotations

import dataclasses
import enum
import json
import logging
import time
from pathlib import Path
from typing import Any

import numpy as np

from sorter.core.io import save_observation
from sorter.core.types import Observation

log = logging.getLogger(__name__)


def to_jsonable(x: Any) -> Any:
    """Dataclasses, enums, and containers → plain JSON values. Arrays are dropped."""
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {
            f.name: to_jsonable(v)
            for f in dataclasses.fields(x)
            if not isinstance(v := getattr(x, f.name), np.ndarray)
        }
    if isinstance(x, enum.Enum):
        return x.value
    if isinstance(x, dict):
        return {str(to_jsonable(k)): to_jsonable(v) for k, v in x.items()}
    if isinstance(x, list | tuple):
        return [to_jsonable(v) for v in x]
    if isinstance(x, np.generic):
        return x.item()
    if isinstance(x, Path):
        return str(x)
    return x


class RunLog:
    def __init__(self, runs_dir: Path, run_id: str):
        self.dir = Path(runs_dir) / run_id
        self.run_id = run_id
        self._info: dict[str, Any] = {"run_id": run_id, "started": _now()}
        self._stems: set[str] = set()
        self._last: tuple[Observation, str] | None = None  # last saved observation, its file

    def start(self, config: dict[str, Any]) -> None:
        self._info["config"] = to_jsonable(config)
        self._write_json("run", self._info)

    def record(self, cycle: int, phase: str, obs: Observation, **data: Any) -> None:
        stem = self._stem(f"{cycle:04d}_{phase}")
        try:
            if self._last is None or self._last[0] is not obs:
                self._last = (obs, save_observation(self.dir / stem, obs).name)
        except OSError as e:
            log.warning("run log: can't save observation %s: %s", stem, e)
        record = {"run_id": self.run_id, "cycle": cycle, "phase": phase, "time": _now()}
        record["observation"] = self._last[1] if self._last else None
        self._write_json(stem, record | to_jsonable(data))

    def finish(self, reason: str, **data: Any) -> None:
        self._info |= {"ended": _now(), "end_reason": reason} | to_jsonable(data)
        self._write_json("run", self._info)

    def _stem(self, base: str) -> str:
        stem, n = base, 1
        while stem in self._stems:
            n += 1
            stem = f"{base}_{n}"
        self._stems.add(stem)
        return stem

    def _write_json(self, stem: str, data: dict[str, Any]) -> None:
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            (self.dir / f"{stem}.json").write_text(json.dumps(data, indent=2))
        except (OSError, TypeError, ValueError) as e:
            log.warning("run log: can't write %s.json: %s", stem, e)


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S")
