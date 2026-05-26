#!/usr/bin/env python3
"""Helpers for recording and plotting primary tool inertia compensation data."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Dict, List, Mapping, MutableMapping, Sequence

SIX_AXIS_LABELS = ["Fx [N]", "Fy [N]", "Fz [N]", "Tx [Nm]", "Ty [Nm]", "Tz [Nm]"]
SOURCE_KEYS = [
    "raw_force_sensor",
    "force_after_gravity_before_inertia",
    "force_after_inertia",
]
SOURCE_LABELS = {
    "raw_force_sensor": "raw_force_sensor",
    "force_after_gravity_before_inertia": "after gravity / before inertia",
    "force_after_inertia": "after inertia",
}
SOURCE_COLORS = {
    "raw_force_sensor": "tab:gray",
    "force_after_gravity_before_inertia": "tab:blue",
    "force_after_inertia": "tab:orange",
}


def default_output_dir() -> Path:
    return Path(__file__).resolve().parent / "inertia_analyse"


def timestamp_filename(now: datetime | None = None) -> str:
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return f"{stamp}.json"


def _float_list(value: object, name: str, min_size: int) -> List[float]:
    if value is None:
        raise ValueError(f"missing {name}")
    try:
        values = [float(item) for item in value]  # type: ignore[union-attr]
    except TypeError as exc:
        raise ValueError(f"{name} must be a numeric list") from exc
    if len(values) < min_size:
        raise ValueError(f"{name} needs at least {min_size} values, got {len(values)}")
    return values


def extract_force_sample(data: Mapping[str, object], arm: str = "primary", t: float = 0.0) -> Dict[str, object]:
    """Extract one primary-arm sample from robot_server_state JSON.

    The current C++ server publishes:
      - raw_force_sensor: raw SRI values, primary in [0:6]
      - force_sensor: final force-control input after gravity and inertia compensation
      - force_sensor_inertia: primary inertia wrench subtracted by the server

    Therefore primary force after gravity and before inertia is reconstructed as:
        force_after_inertia + force_sensor_inertia
    """
    if arm != "primary":
        raise ValueError("only primary arm is supported")

    raw_all = _float_list(data.get("raw_force_sensor"), "raw_force_sensor", 6)
    force_all = _float_list(data.get("force_sensor"), "force_sensor", 6)
    inertia = _float_list(data.get("force_sensor_inertia", [0.0] * 6), "force_sensor_inertia", 6)

    raw = raw_all[:6]
    after_inertia = force_all[:6]
    inertia = inertia[:6]
    before_inertia = [after_inertia[i] + inertia[i] for i in range(6)]

    sample: Dict[str, object] = {
        "t": float(t),
        "timestamp": data.get("timestamp"),
        "raw_force_sensor": raw,
        "force_after_gravity_before_inertia": before_inertia,
        "force_after_inertia": after_inertia,
        "force_sensor_inertia": inertia,
    }
    return sample


def make_payload(
    samples: Sequence[Mapping[str, object]],
    *,
    state_topic: str,
    created_at: datetime | None = None,
) -> MutableMapping[str, object]:
    created = created_at or datetime.now()
    return {
        "metadata": {
            "created_at": created.isoformat(timespec="seconds"),
            "state_topic": state_topic,
            "arm": "primary",
            "sample_count": len(samples),
            "sources": SOURCE_KEYS,
            "axis_labels": SIX_AXIS_LABELS,
            "reconstruction": (
                "force_after_gravity_before_inertia = "
                "force_after_inertia + force_sensor_inertia"
            ),
            "raw_force_sensor_note": (
                "raw_force_sensor is stored exactly as published by the C++ server "
                "(primary slice [0:6])."
            ),
        },
        "samples": list(samples),
    }


def source_matrix(samples: Sequence[Mapping[str, object]], key: str) -> List[List[float]]:
    matrix: List[List[float]] = []
    for sample in samples:
        values = _float_list(sample.get(key), key, 6)
        matrix.append(values[:6])
    return matrix
