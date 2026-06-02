#!/usr/bin/env python3
"""CLI for CloudSim canonical program export."""

import argparse
import sys
from pathlib import Path

from read_canonical_v1 import load_canonical_v1, walk_tree


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="CloudSim program export postprocess")
    parser.add_argument("input", type=Path, help="*.cloudsim-program.json")
    parser.add_argument("--print-summary", action="store_true", help="Print program tree summary")
    parser.add_argument("--brand", choices=["abb", "kuka"], help="Run brand emitter stub")
    args = parser.parse_args(argv)

    doc = load_canonical_v1(args.input)
    if args.print_summary:
        print(f"program={doc.program.get('name')} layout={doc.export_layout}")
        print(f"motion_refs={len(doc.flat_motion_sequence)}")
        walk_tree(doc.instructions)

    if args.brand == "abb":
        from emit_abb import emit_rapid_stub

        print(emit_rapid_stub(doc))
    elif args.brand == "kuka":
        from emit_kuka import emit_src_stub

        print(emit_src_stub(doc))

    return 0


if __name__ == "__main__":
    sys.exit(main())
