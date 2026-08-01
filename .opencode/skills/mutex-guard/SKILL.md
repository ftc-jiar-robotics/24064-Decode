---
name: mutex-guard
description: Use when editing Subsystem, RobotActions, ActionScheduler, or any robot subsystem / opmode code, and before committing or pushing, to verify the Subsystem mutex system was not invalidated. Reads the contract docs and source files and reasons about violations directly (no script). Triggers on "mutex", "politeSet", "forceSet", "setLocked", "onSet", or "code guidelines check".
---

# Mutex Guard

Verifies the `Subsystem<T>` mutex contract by **reading the actual source and
reasoning** about whether each file respects the privilege boundary. The
contract lives in `MUTEX.md` and `AGENTS.md`; do not rely on the regex scanner
in `tools/mutex_check.py` — it is a CI backstop and misses structural bypasses.

The team's root package name changes every season (this year it is `decode`).
Never assume it: always locate the subsystem package dynamically.

## When to run

- after editing the Subsystem base class, RobotActions, ActionScheduler, or any
  subsystem / opmode code,
- **before every commit or push** (CI runs the same contract), and
- when asked to fix "mutex" or "MUTEX CONTRACT VIOLATIONS".

## Step 1 — Locate the code and load the contract

1. Read `MUTEX.md` — the authoritative contract (access table, rules). Its
   rules are package-agnostic; only its example paths reference this year's
   package.
2. Read `AGENTS.md` — the "Repo-specific gotchas" and "Workflow" sections.
3. Locate the team source root: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`
   (the `org.firstinspires.ftc.teamcode` part is stable).
4. Use Glob to find `**/subsystem/Subsystem.java` under that root. Its parent
   directory defines the **privileged package** for this year (e.g.
   `<team>/subsystem`, where `<team>` is `decode` this season). Read that file —
   it is the ground truth for the API surface: `politeSet` (public final),
   `forceSet` (package-private final), `onSet` (package-private abstract),
   `setLocked` (package-private).

Internalize the boundary:

| Method          | Visible to                | Callable by                                  |
|-----------------|---------------------------|----------------------------------------------|
| `politeSet(T)`  | everyone                  | anyone — `false` while locked                |
| `forceSet(T)`   | privileged package only   | actions / composites in that package         |
| `onSet(T)`      | privileged package only   | **base class only** — subclasses implement, never call |
| `setLocked(boolean)` | privileged package only | scheduler path (and Shooter's fan-out)    |

## Step 2 — Enumerate and partition the code

Use Glob to list every `.java` under the team source root, then partition:

- the privileged package (the one holding `Subsystem.java`) — may call
  `forceSet` / `setLocked`;
- every other team package (`<team>.opmodes`, `<team>.util`, ...) —
  unprivileged.

## Step 3 — Reason over each file

For **every** `.java` in the partition, check with Read/Grep:

1. **Subsystem subclasses** (`extends Subsystem`):
   - implements `onSet(...)` as package-private (no modifier, not `public`),
     and it only writes that subsystem's own private state field;
   - does **not** declare `politeSet` / `forceSet` (both `final` in base);
   - does **not** override `setLocked` unless it is a composite like Shooter
     that fans the lock out to children — and that override must stay
     package-private too;
   - exposes **no public method that mutates its state field directly**
     (that is a lock bypass even if `politeSet` is untouched);
   - its state field (`currentState`, `targetState`, `power`, `targetAngle`,
     ...) is `private`, so nothing outside the class can write it.

2. **Privileged callers** (the subsystem package): every `forceSet(...)` /
   `setLocked(...)` / `onSet(...)` reference must be an internal, justified use
   (an action in RobotActions, the scheduler in ActionScheduler, a composite
   fan-out in Shooter, or the base class itself). Any use that reads like an
   OpMode or an ad-hoc shortcut is a violation.

3. **Unprivileged files** (opmodes, util, anything outside the subsystem
   package): must reference subsystems **only** through `politeSet`, `get()`,
   and read-only `getXxx()` accessors. Any `forceSet`, `setLocked`, `onSet`,
   or old `set(...)` call here is a violation.

4. **Structural bypasses** the regex scanner cannot see:
   - a subsystem that gained a `public void` / `public boolean` setter-like
     method writing its state field;
   - state fields accidentally made package-private or public;
   - a composite exposing children publicly such that an opmode could
     `robot.shooter.flywheel.forceSet(...)` (the force path must stay inside
     the privileged package regardless of reachability);
   - reflection (`Class.forName(...)`, `getDeclaredMethod(...)`) reaching the
     package-private API;
   - `onSet` invoked through a helper or method reference (`this::onSet`,
     `subsystem::onSet`).

## Step 4 — Report

For every violation, cite `file:line`, the specific contract rule from
`MUTEX.md` it breaks, and the offending line. Then fix each one — prefer the
correct privilege path (`politeSet` for OpModes, `forceSet` inside the
privileged package) over changing the access modifier. Never weaken the
contract to accommodate a caller.

Example report entry:

```
subsystem/Turret.java:412: onSet called on instance -- rule 1. onSet is a
    base-class hook; only Subsystem.politeSet/forceSet may invoke it.
```

## Step 5 — Close the loop

- Re-run the review after fixes; it must find nothing.
- Cross-check with CI parity: `python3 tools/mutex_check.py .` (must exit 0).
  The script derives the privileged package the same way (by locating
  `subsystem/Subsystem.java`), so it stays correct after a yearly rename. If it
  flags something the reasoning pass considers fine, re-read `MUTEX.md` — the
  scanner is strict on purpose.
- Compile with `./gradlew assembleDebug`.
- Then commit/push; the `mutex-guard` GitHub Actions workflow runs the same
  contract and fails the check-in on any regression.
