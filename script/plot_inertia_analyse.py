#!/usr/bin/env python3
"""Plot primary force data recorded by record_inertia_analyse.py.

Each subplot corresponds to one wrench dimension and overlays:
raw_force_sensor, force_after_gravity_before_inertia, and force_after_inertia.
Use the Matplotlib toolbar to zoom/pan, and the checkboxes to toggle sources.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Mapping

from inertia_analyse_utils import (
    SIX_AXIS_LABELS,
    SOURCE_COLORS,
    SOURCE_KEYS,
    SOURCE_LABELS,
    default_output_dir,
    source_matrix,
)


def latest_recording(output_dir: Path) -> Path:
    candidates = sorted(output_dir.glob("*.json"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"no JSON recordings found in {output_dir}")
    return candidates[-1]


def load_payload(path: Path) -> Mapping[str, object]:
    with path.open("r", encoding="utf-8") as f:
        payload = json.load(f)
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"{path} has no samples")
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot force data recorded for tool inertia analysis.")
    parser.add_argument("path", nargs="?", type=Path, help="recording JSON; default is latest in script/inertia_analyse")
    parser.add_argument("--output-dir", type=Path, default=default_output_dir(), help="directory used when path is omitted")
    parser.add_argument(
        "--show",
        nargs="+",
        choices=SOURCE_KEYS,
        default=SOURCE_KEYS,
        help="sources visible initially; checkboxes can toggle them after the window opens",
    )
    return parser.parse_args()


def update_ylim(ax, line_list: List[object]) -> None:
    ymin = None
    ymax = None
    for line in line_list:
        if not line.get_visible():
            continue
        ydata = line.get_ydata()
        finite = [float(y) for y in ydata if y == y and y not in (float("inf"), float("-inf"))]
        if not finite:
            continue
        low = min(finite)
        high = max(finite)
        ymin = low if ymin is None else min(ymin, low)
        ymax = high if ymax is None else max(ymax, high)
    if ymin is None or ymax is None or abs(ymax - ymin) < 1e-9:
        ymin, ymax = -1.0, 1.0
    pad = 0.1 * (ymax - ymin) + 1e-3
    ax.set_ylim(ymin - pad, ymax + pad)


def main() -> None:
    args = parse_args()

    import matplotlib.pyplot as plt
    from matplotlib.widgets import CheckButtons

    path = args.path or latest_recording(args.output_dir)
    payload = load_payload(path)
    samples = payload["samples"]

    t = [float(sample.get("t", idx)) for idx, sample in enumerate(samples)]
    matrices: Dict[str, List[List[float]]] = {
        key: source_matrix(samples, key)
        for key in SOURCE_KEYS
    }

    fig, axes_grid = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
    axes = list(axes_grid.flatten())
    lines_by_source: Dict[str, List[plt.Line2D]] = {key: [] for key in SOURCE_KEYS}
    lines_by_axis: List[List[plt.Line2D]] = [[] for _ in range(6)]
    initially_visible = set(args.show)

    for axis_idx, ax in enumerate(axes):
        for source in SOURCE_KEYS:
            y = [row[axis_idx] for row in matrices[source]]
            (line,) = ax.plot(
                t,
                y,
                label=SOURCE_LABELS[source],
                color=SOURCE_COLORS[source],
                linewidth=1.25 if source != "raw_force_sensor" else 1.0,
                linestyle="--" if source == "raw_force_sensor" else "-",
                alpha=0.85,
                visible=source in initially_visible,
            )
            lines_by_source[source].append(line)
            lines_by_axis[axis_idx].append(line)
        ax.set_title(SIX_AXIS_LABELS[axis_idx])
        ax.set_xlabel("time [s]")
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color="gray", linewidth=0.6, alpha=0.5)
        ax.legend(loc="upper left", fontsize=8)
        update_ylim(ax, lines_by_axis[axis_idx])

    check_ax = fig.add_axes([0.83, 0.02, 0.15, 0.12])
    check = CheckButtons(
        check_ax,
        [SOURCE_LABELS[key] for key in SOURCE_KEYS],
        [key in initially_visible for key in SOURCE_KEYS],
    )

    def on_check(label: str) -> None:
        source = next(key for key in SOURCE_KEYS if SOURCE_LABELS[key] == label)
        for line in lines_by_source[source]:
            line.set_visible(not line.get_visible())
        for idx, ax in enumerate(axes):
            update_ylim(ax, lines_by_axis[idx])
        fig.canvas.draw_idle()

    check.on_clicked(on_check)

    title = f"primary inertia analyse: {path.name}"
    metadata = payload.get("metadata", {})
    if isinstance(metadata, dict) and metadata.get("sample_count") is not None:
        title += f" ({metadata['sample_count']} samples)"
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    plt.show()


if __name__ == "__main__":
    main()
