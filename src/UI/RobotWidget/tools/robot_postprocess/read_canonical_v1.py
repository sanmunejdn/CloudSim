"""Read cloudsim.program_export v1 JSON for brand-specific postprocessors."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator


FORMAT_ID = "cloudsim.program_export"
SCHEMA_VERSION = 1


@dataclass
class FlatMotionRef:
    flat_index: int
    instruction_id: str
    program_step_path: list[int]
    point_index: int = 0


@dataclass
class CanonicalProgramV1:
    format: str
    schema_version: int
    export_layout: str
    program: dict[str, Any]
    robot: dict[str, Any]
    coordinate_frames: dict[str, Any]
    instructions: list[dict[str, Any]]
    flat_motion_sequence: list[FlatMotionRef] = field(default_factory=list)


def load_canonical_v1(path: str | Path) -> CanonicalProgramV1:
    path = Path(path)
    with path.open(encoding="utf-8") as f:
        root = json.load(f)
    if root.get("format") != FORMAT_ID:
        raise ValueError(f"unexpected format: {root.get('format')}")
    if int(root.get("schemaVersion", 0)) != SCHEMA_VERSION:
        raise ValueError(f"unsupported schemaVersion: {root.get('schemaVersion')}")
    flat = []
    for item in root.get("flatMotionSequence", []):
        flat.append(
            FlatMotionRef(
                flat_index=int(item.get("flatIndex", 0)),
                instruction_id=str(item.get("instructionId", "")),
                program_step_path=[int(x) for x in item.get("programStepPath", [])],
                point_index=int(item.get("pointIndex", 0)),
            )
        )
    return CanonicalProgramV1(
        format=root["format"],
        schema_version=int(root["schemaVersion"]),
        export_layout=str(root.get("exportLayout", "nested_tree")),
        program=dict(root.get("program", {})),
        robot=dict(root.get("robot", {})),
        coordinate_frames=dict(root.get("coordinateFrames", {})),
        instructions=list(root.get("instructions", [])),
        flat_motion_sequence=flat,
    )


def iter_nested(records: list[dict[str, Any]]) -> Iterator[dict[str, Any]]:
    """Depth-first over nested_tree instructions (IF then/else, WHILE body)."""
    for rec in records:
        yield rec
        if rec.get("type") == "if":
            yield from iter_nested(rec.get("then", []))
            yield from iter_nested(rec.get("else", []))
        elif rec.get("type") == "while":
            yield from iter_nested(rec.get("body", []))


def iter_flat_motion(doc: CanonicalProgramV1) -> Iterator[dict[str, Any]]:
    """Motion leaves in DFS order (matches flatMotionSequence)."""
    by_id = {node["instructionId"]: node for node in iter_nested(doc.instructions) if node.get("instructionId")}
    for ref in doc.flat_motion_sequence:
        node = by_id.get(ref.instruction_id)
        if node:
            yield node


def walk_tree(records: list[dict[str, Any]], indent: int = 0) -> None:
    for rec in records:
        t = rec.get("type", "?")
        name = rec.get("name", "")
        print("  " * indent + f"{t} {name}")
        if t == "if":
            print("  " * indent + "  THEN:")
            walk_tree(rec.get("then", []), indent + 2)
            print("  " * indent + "  ELSE:")
            walk_tree(rec.get("else", []), indent + 2)
        elif t == "while":
            walk_tree(rec.get("body", []), indent + 1)
