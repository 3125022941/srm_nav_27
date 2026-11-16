#!/usr/bin/env python3
"""Give every leg a unique name so repeated routes are measured separately."""
import argparse
import copy
import json
from pathlib import Path


def prepare(scene, output, rounds):
    if rounds < 1:
        raise ValueError("rounds must be positive")
    manifest_file = scene / "scene_manifest.json"
    original = manifest_file.read_bytes()
    manifest = json.loads(original)
    output.mkdir(parents=True, exist_ok=True)
    (output / "scene_manifest_original.json").write_bytes(original)
    home = manifest["spawn_xy"]
    manifest["goals"] = [
        {"name": f"cycle_{cycle:02d}_{direction}", "xy": copy.copy(xy)}
        for cycle in range(1, rounds + 1)
        for direction, xy in [("out", [0.245, 0.585]), ("home", home)]
    ]
    manifest["validation_note"] = (
        "Room-internal out-and-back route; corridor exits were disconnected "
        "in the 2026-11-16 global costmap snapshot."
    )
    encoded = json.dumps(manifest, indent=2) + "\n"
    (output / "loop_manifest.json").write_text(encoded, encoding="utf-8")
    manifest_file.write_text(encoded, encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--rounds", type=int, default=5)
    args = parser.parse_args()
    if args.rounds < 1:
        parser.error("--rounds must be positive")
    prepare(args.scene, args.out, args.rounds)
