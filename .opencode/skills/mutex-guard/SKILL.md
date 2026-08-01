---
name: mutex-guard
description: Use when editing Subsystem, RobotActions, ActionScheduler, or any robot subsystem / opmode code, and before committing or pushing, to verify the Subsystem mutex system was not invalidated. Triggers on "mutex", "politeSet", "forceSet", "setLocked", "onSet", or "code guidelines check".
---

# Mutex Guard

Verifies the `Subsystem<T>` mutex contract is intact using the static scanner
at `tools/mutex_check.py`. The full contract is in `MUTEX.md`.

## When to run

Run this check:

- after editing any file under `TeamCode/.../decode/subsystem/`,
- after editing any opmode that touches a subsystem,
- **before every commit or push** (CI enforces the same check), and
- when asked to fix "mutex" or "MUTEX CONTRACT VIOLATIONS".

## Procedure

1. Run the check from the repo root:

   ```bash
   python3 tools/mutex_check.py .
   ```

2. `MUTEX CONTRACT OK` — nothing to do. Pass it on to CI (`mutex-guard`
   workflow) and commit.

3. `MUTEX CONTRACT VIOLATIONS` — every line is `file:line: reason` plus the
   offending source line. Fix each one, then re-run until it exits 0.

## Violation fixes

| Reported rule                     | Fix                                                                 |
|-----------------------------------|---------------------------------------------------------------------|
| `onSet(...)` called on instance   | `onSet` is the base-class hook. Don't call it. Subclasses only *implement* it; OpModes/Actions must use `politeSet`/`forceSet`. |
| `forceSet(...)` outside package   | Only `decode.subsystem` code (actions) may force. OpModes must use `politeSet`. If an action needs it, keep it in `decode.subsystem`. |
| `setLocked(...)` outside package  | Only the scheduler path in `decode.subsystem` may lock/unlock.       |
| mutex API widened to `public`     | `onSet` / `setLocked` / `forceSet` overrides must stay package-private (no modifier). |
| `politeSet`/`forceSet` declared   | They are `final` in `Subsystem.java`; never re-declare them.        |
| old `set(...)` API used           | Use `politeSet` (OpModes) or `forceSet` (actions). `setXxx(...)` helpers are unrelated and fine. |

If the check flags something that looks legitimate, re-read `MUTEX.md` — the
scanner is intentionally strict and the contract is the source of truth. Never
weaken the scanner to silence a real violation.

## After fixing

- Re-run `python3 tools/mutex_check.py .` — must exit 0.
- Compile with `./gradlew assembleDebug`.
- Then commit/push; the `mutex-guard` GitHub Actions workflow will run the same
  check and fail the check-in if it regresses.
