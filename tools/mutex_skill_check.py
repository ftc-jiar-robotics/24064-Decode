#!/usr/bin/env python3
"""Validate the mutex-guard skill ecosystem and the docs-vs-code contract.

The mutex-guard *skill* reasons over sources using MUTEX.md as the contract and
Subsystem.java as ground truth. An LLM can't run in CI, so this script statically
asserts the same invariants the skill depends on:

  1. Every artifact the skill references exists: MUTEX.md, AGENTS.md,
     tools/mutex_check.py, .opencode/skills/mutex-guard/SKILL.md,
     .github/workflows/mutex-guard.yml.
  2. SKILL.md is well-formed (frontmatter name matches its folder).
  3. AGENTS.md points at the check and the contract doc.
  4. Subsystem.java's mutex API surface exactly matches the documented contract
     (politeSet public final, forceSet package-private final, onSet package-private
     abstract, setLocked package-private) -- located dynamically, so a yearly
     rename of the team package does not break it.
  5. Robot.java lives in its own <team>/robot package (NOT the subsystem
     package), so it cannot reach the package-private mutex API.
  6. The workflow actually runs both mutex checks.

Usage:
    python3 tools/mutex_skill_check.py [repo-root]
Exits 0 on pass, 1 on any violation.
"""

import re
import sys
from pathlib import Path

CONTRACT = [
    # (must_be_present, must_be_absent, label)
    ("public final boolean politeSet(T t)", None,
     "politeSet must be public final (OpMode path)"),
    ("final boolean forceSet(T t)", "public final boolean forceSet",
     "forceSet must be package-private final (actions only)"),
    ("abstract void onSet(T t)", "public abstract void onSet",
     "onSet must be package-private abstract (base-class hook)"),
    ("void setLocked(boolean", "public void setLocked",
     "setLocked must be package-private (scheduler only)"),
]


def must_exist(path: Path, label: str) -> list:
    return [] if path.exists() else [f"missing: {label} ({path})"]


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    problems = []

    # 1. Artifact presence.
    for rel, label in [
        (Path("MUTEX.md"), "MUTEX.md contract doc"),
        (Path("AGENTS.md"), "AGENTS.md agent guide"),
        (Path("tools/mutex_check.py"), "tools/mutex_check.py scanner"),
        (Path(".opencode/skills/mutex-guard/SKILL.md"), "mutex-guard skill"),
        (Path(".github/workflows/mutex-guard.yml"), "mutex-guard workflow"),
    ]:
        problems += must_exist(root / rel, label)

    # 2. SKILL.md frontmatter: name must match its folder.
    skill = root / ".opencode/skills/mutex-guard/SKILL.md"
    if skill.exists():
        text = skill.read_text(encoding="utf-8")
        m = re.search(r"^name:\s*(.+)$", text, re.MULTILINE)
        if not m or m.group(1).strip() != "mutex-guard":
            problems.append("SKILL.md frontmatter name must be 'mutex-guard' to match its folder")
        if "description:" not in text:
            problems.append("SKILL.md is missing its description frontmatter")

    # 3. AGENTS.md must point at the check + contract.
    agents = root / "AGENTS.md"
    if agents.exists():
        text = agents.read_text(encoding="utf-8")
        if "tools/mutex_check.py" not in text:
            problems.append("AGENTS.md no longer references tools/mutex_check.py")
        if "MUTEX.md" not in text:
            problems.append("AGENTS.md no longer references MUTEX.md")

    # 4. Subsystem.java surface matches the documented contract.
    scan_root = (root / "TeamCode" / "src" / "main" / "java"
                 / "org" / "firstinspires" / "ftc" / "teamcode")
    base = next(scan_root.rglob("subsystem/Subsystem.java"), None) if scan_root.exists() else None
    if base is None:
        problems.append(f"could not locate subsystem/Subsystem.java under {scan_root}")
    else:
        src = base.read_text(encoding="utf-8")
        for present, absent, label in CONTRACT:
            if present not in src:
                problems.append(f"{base.relative_to(root)}: {label} -- expected '{present}'")
            if absent and absent in src:
                problems.append(f"{base.relative_to(root)}: {label} -- found forbidden '{absent}'")

    # 5. Robot.java must live outside the subsystem package.
    robots = list(scan_root.rglob("Robot.java")) if scan_root.exists() else []
    if len(robots) != 1:
        problems.append(f"expected exactly one Robot.java under {scan_root}, found {len(robots)}")
    elif base is not None and robots[0].parent == base.parent:
        problems.append("Robot.java must not live in the subsystem package -- it is the "
                        "opmode-facing composition root and must be kept out of the mutex boundary")

    # 6. Workflow runs both checks.
    wf = root / ".github/workflows/mutex-guard.yml"
    if wf.exists():
        text = wf.read_text(encoding="utf-8")
        for script in ("tools/mutex_check.py", "tools/mutex_skill_check.py"):
            if script not in text:
                problems.append(f"mutex-guard.yml must run {script}")

    if problems:
        print("MUTEX SKILL/DOCS CHECK FAILED")
        print("-----------------------------")
        print("\n".join(problems))
        print("\nFix the drift above, then re-run.")
        return 1

    print("MUTEX SKILL/DOCS CHECK OK -- skill ecosystem and contract are consistent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
