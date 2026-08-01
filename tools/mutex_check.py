#!/usr/bin/env python3
"""Static guard for the Subsystem mutex contract (Team 24064 / DECODE).

The mutex system lets RobotActions + ActionScheduler (inside decode.subsystem)
lock subsystems and force-set their state, while OpModes (decode.opmodes) can
only make lock-respecting "polite" requests. This script rejects any change that
invalidates that privilege boundary. See MUTEX.md for the full contract.

Invariants enforced:
  1. onSet()    is a base-class-only state hook -- never called on an instance.
  2. forceSet() is package-private -- callable only from decode.subsystem.
  3. setLocked() is package-private -- callable only from decode.subsystem.
  4. onSet / setLocked / forceSet are never widened to public.
  5. politeSet / forceSet are never re-declared outside Subsystem.java.
  6. The removed old API (subsystem.set(...)) is never reintroduced.

Usage:
    python3 tools/mutex_check.py [repo-root]
Exits 0 on pass, 1 on any violation.
"""

import re
import sys
from pathlib import Path

SUBSYSTEM_VARS = (
    "intake", "feeder", "shooter", "flywheel", "turret", "hood",
    "gateOpener", "parkLift", "tilt",
)

# Each rule: pattern, human message, and a predicate (is_base_file, is_subsystem_pkg)
# that says whether a match is legal. Rules 1/4/6 apply to every file.
RULES = [
    {"pattern": re.compile(r"\.onSet\("),
     "message": "onSet is the base-class state hook; it must never be called on an instance",
     "legal": lambda base, pkg: False},
    {"pattern": re.compile(r"\.forceSet\("),
     "message": "forceSet is package-private; only decode.subsystem may call it",
     "legal": lambda base, pkg: pkg},
    {"pattern": re.compile(r"\.setLocked\("),
     "message": "setLocked is package-private; only decode.subsystem may call it",
     "legal": lambda base, pkg: pkg},
    {"pattern": re.compile(r"\bpublic\b[^\n]*\b(onSet|setLocked|forceSet)\s*\("),
     "message": "package-private mutex API was widened to public",
     "legal": lambda base, pkg: False},
    {"pattern": re.compile(r"\b(?:boolean|void)\s+(politeSet|forceSet)\s*\("),
     "message": "politeSet/forceSet must not be declared outside Subsystem.java",
     "legal": lambda base, pkg: base},
    {"pattern": re.compile(r"\b(" + "|".join(SUBSYSTEM_VARS) + r")\.set\("),
     "message": "regression to the removed subsystem set() API (use politeSet / forceSet)",
     "legal": lambda base, pkg: False},
]


def is_comment_or_blank(line: str) -> bool:
    s = line.lstrip()
    return not s or s.startswith(("//", "*"))


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    scan_dir = (root / "TeamCode" / "src" / "main" / "java"
                / "org" / "firstinspires" / "ftc" / "teamcode")
    violations = []

    for java_file in sorted(scan_dir.rglob("*.java")):
        rel = java_file.relative_to(scan_dir)
        in_subsystem_pkg = (rel.parts[0] == "decode" and rel.parts[1] == "subsystem")
        is_base = in_subsystem_pkg and java_file.name == "Subsystem.java"

        for lineno, raw in enumerate(java_file.read_text(encoding="utf-8").splitlines(), 1):
            if is_comment_or_blank(raw):
                continue
            line = raw.split("//", 1)[0]
            for rule in RULES:
                if rule["pattern"].search(line) and not rule["legal"](is_base, in_subsystem_pkg):
                    violations.append(f"{rel}:{lineno}: {rule['message']}\n    {raw.strip()}")

    if violations:
        print("MUTEX CONTRACT VIOLATIONS")
        print("-------------------------")
        print("\n".join(violations))
        print("\nBlocked: fix the violations above, or this gate will keep failing. See MUTEX.md.")
        return 1

    print("MUTEX CONTRACT OK -- no invalidations found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
