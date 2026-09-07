#!/usr/bin/env python3
"""Merge every colcon package's compile_commands.json into one at the workspace
root, which is where clangd looks for it. Re-run after adding files/packages."""
import json
from pathlib import Path

root = Path(__file__).resolve().parent.parent
entries, seen = [], set()
for db in sorted(root.glob("build/*/compile_commands.json")):
    for e in json.loads(db.read_text()):
        key = (e.get("directory"), e.get("file"))
        if key not in seen:
            seen.add(key)
            entries.append(e)
out = root / "compile_commands.json"
out.write_text(json.dumps(entries, indent=2) + "\n")
print(f"wrote {out} ({len(entries)} entries)")
