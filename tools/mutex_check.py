#!/usr/bin/env python3
"""Static guard for the Subsystem mutex contract (Team 24064 / DECODE).

The mutex system lets RobotActions + ActionScheduler (inside the subsystem
package) lock subsystems and force-set their state, while OpModes (other team
packages) can only make lock-respecting "polite" requests. Robot.java is the
opmode-facing composition root and lives in its own <team>/robot package, so it
is NOT part of the mutex machinery: it must never force a state or take a lock.
This script rejects any change that invalidates that privilege boundary. See
MUTEX.md.

The team's root package is NOT hardcoded: the subsystem package is located by
finding Subsystem.java under teamcode/, so the guard survives yearly renames.

Invariants enforced:
    1. onSet_DONOTCALL() is a base-class-only state hook -- never called on an
       instance.
    2. forceSet() is package-private -- callable only from the subsystem package.
    3. setLocked() is package-private -- callable only from the subsystem package.
    4. onSet_DONOTCALL / setLocked / forceSet are never widened to public.
  5. politeSet / forceSet are never re-declared outside Subsystem.java.
  6. The removed old API (subsystem.set(...)) is never reintroduced.
  7. Robot.java lives outside the subsystem package and never calls forceSet /
     setLocked (it must request state only through politeSet).
  8. Subsystem subclasses expose no public mutator methods (setXxx / clearXxx /
     resetXxx / toggleXxx / applyXxx / enableXxx / disableXxx / incrementXxx /
     decrementXxx) unless listed in tools/mutex_whitelist.txt as
     ClassName::methodName. Whitelisted entries are fail-safe (lock-aware) or
     configuration-type only; anything state-changing belongs behind a
     RobotAction, package-private.

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

# Rule 8: public mutator methods on Subsystem subclasses. The whitelist lives
# at <root>/tools/mutex_whitelist.txt and is loaded at runtime.
MUTATOR_NAMES = (
    "set", "clear", "reset", "toggle", "apply", "enable", "disable",
    "increment", "decrement",
)
PUBLIC_MUTATOR_RE = re.compile(
    r"\bpublic\b[^\n]*\b(" + "|".join(MUTATOR_NAMES) + r")([A-Z]\w*)\s*\(")
SUBCLASS_RE = re.compile(r"class\s+(\w+)\s+extends\s+Subsystem\b")

# Each rule: pattern, human message, and a predicate (is_base_file, is_subsystem_pkg)
# that says whether a match is legal. Rules 1/4/6 apply to every file.
RULES = [
    {"pattern": re.compile(r"\.onSet_DONOTCALL\("),
     "message": "onSet_DONOTCALL is the base-class state hook; it must never be called on an instance",
     "legal": lambda base, pkg, robot: False},
    {"pattern": re.compile(r"\.forceSet\("),
     "message": "forceSet is package-private; only the subsystem package may call it",
     "legal": lambda base, pkg, robot: pkg and not robot},
    {"pattern": re.compile(r"\.setLocked\("),
     "message": "setLocked is package-private; only the subsystem package may call it",
     "legal": lambda base, pkg, robot: pkg and not robot},
    {"pattern": re.compile(r"\bpublic\b[^\n]*\b(onSet_DONOTCALL|setLocked|forceSet)\s*\("),
     "message": "package-private mutex API was widened to public",
     "legal": lambda base, pkg, robot: False},
    {"pattern": re.compile(r"\b(?:boolean|void)\s+(politeSet|forceSet)\s*\("),
     "message": "politeSet/forceSet must not be declared outside Subsystem.java",
     "legal": lambda base, pkg, robot: base},
    {"pattern": re.compile(r"\b(" + "|".join(SUBSYSTEM_VARS) + r")\.set\("),
     "message": "regression to the removed subsystem set() API (use politeSet / forceSet)",
     "legal": lambda base, pkg, robot: False},
]


def is_comment_or_blank(line: str) -> bool:
    s = line.lstrip()
    return not s or s.startswith(("//", "*"))


def find_subsystem_dir(scan_dir: Path) -> Path:
    """Return the dir holding Subsystem.java (e.g. <team>/subsystem), or None."""
    for candidate in scan_dir.rglob("subsystem/Subsystem.java"):
        return candidate.parent
    return None


def load_whitelist(whitelist_path: Path) -> set:
    """Load `ClassName::methodName` whitelist entries (skip blanks/# comments)."""
    entries = set()
    if not whitelist_path.exists():
        return entries
    for raw in whitelist_path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            entries.add(line)
    return entries


def main() -> int:
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    scan_dir = (root / "TeamCode" / "src" / "main" / "java"
                / "org" / "firstinspires" / "ftc" / "teamcode")
    subsystem_dir = find_subsystem_dir(scan_dir)
    if subsystem_dir is None:
        print("MUTEX CONTRACT BLOCKED: could not locate subsystem/Subsystem.java "
              "under teamcode/. Cannot determine the privilege boundary.")
        return 1
    subsystem_parts = subsystem_dir.relative_to(scan_dir).parts

    whitelist = load_whitelist(root / "tools" / "mutex_whitelist.txt")

    # Robot.java is the opmode-facing composition root. It must live in its own
    # <team>/robot package (NOT the subsystem package) so it has no mutex power.
    robot_files = list(scan_dir.rglob("Robot.java"))
    if len(robot_files) != 1:
        print(f"MUTEX CONTRACT BLOCKED: expected exactly one Robot.java under teamcode/, "
              f"found {len(robot_files)}.")
        return 1
    robot_file = robot_files[0]
    robot_parts = robot_file.parent.relative_to(scan_dir).parts
    if robot_parts == subsystem_parts:
        print("MUTEX CONTRACT VIOLATION: Robot.java lives in the subsystem package. "
              "Move it to its own <team>/robot package so it cannot call forceSet/setLocked.")
        return 1

    violations = []
    for java_file in sorted(scan_dir.rglob("*.java")):
        rel = java_file.relative_to(scan_dir)
        in_subsystem_pkg = rel.parts[:len(subsystem_parts)] == subsystem_parts
        is_base = java_file == subsystem_dir / "Subsystem.java"
        is_robot = java_file == robot_file

        # Rule 8: this file's class name, if it declares a Subsystem subclass.
        subclass_name = None
        if not is_base:
            m = SUBCLASS_RE.search(java_file.read_text(encoding="utf-8"))
            if m:
                subclass_name = m.group(1)

        for lineno, raw in enumerate(java_file.read_text(encoding="utf-8").splitlines(), 1):
            if is_comment_or_blank(raw):
                continue
            line = raw.split("//", 1)[0]
            for rule in RULES:
                if rule["pattern"].search(line) and not rule["legal"](is_base, in_subsystem_pkg, is_robot):
                    violations.append(f"{rel}:{lineno}: {rule['message']}\n    {raw.strip()}")

            if subclass_name:
                mm = PUBLIC_MUTATOR_RE.search(line)
                if mm:
                    method = mm.group(1) + mm.group(2)
                    if f"{subclass_name}::{method}" not in whitelist:
                        violations.append(
                            f"{rel}:{lineno}: public mutator {subclass_name}::{method}() "
                            f"bypasses the mutex -- make it package-private behind a "
                            f"RobotAction, or whitelist it in tools/mutex_whitelist.txt "
                            f"(fail-safe/configuration-type only)\n    {raw.strip()}")

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
