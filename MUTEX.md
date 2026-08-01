# Mutex System — Code Guidelines & Enforcement

The `Subsystem<T>` base class (`decode/subsystem/Subsystem.java`) implements a
mutex so that **only RobotActions / ActionScheduler** may force subsystems into a
state while an action is running, while OpModes only get a lock-respecting
request that silently no-ops while the subsystem is locked.

## The contract

| Method            | Visibility                  | Who may call                                  |
|-------------------|-----------------------------|-----------------------------------------------|
| `politeSet(T)`    | `public final`              | Anyone — rejected (`false`) while locked      |
| `forceSet(T)`     | package-private `final`     | Only `decode.subsystem` (actions)             |
| `onSet(T)`        | package-private `abstract`  | Base class only (never call it)               |
| `setLocked(boolean)` | package-private           | Only `decode.subsystem` (scheduler)           |

Key rules:

1. **`onSet` is a base-class-only hook.** Subclasses *implement* it; nothing
   else may *call* it. A call like `intake.onSet(...)` bypasses the lock.
2. **`forceSet` is package-private.** Only code whose package is
   `decode.subsystem` (`Shooter`, `RobotActions`, `ActionScheduler`, ...) may
   force a state. OpModes in `decode.opmodes` must use `politeSet`.
3. **`setLocked` is package-private.** Only the scheduler path may acquire or
   release the lock. Shooter fans the lock out to its children via its own
   package-private override.
4. **Never widen the mutex API.** `onSet`, `forceSet`, `setLocked` overrides
   must stay package-private. A `public` override exposes the mutex to OpModes.
5. **Never re-declare `politeSet` / `forceSet`.** They are `final`; a subclass
   declaring them breaks the base-class enforcement.
6. **Never use the old `set(...)` API** on a subsystem. It was removed. Use
   `politeSet` (OpModes) or `forceSet` (actions). Note `setXxx(...)` helpers
   (`setQueuedShots`, `setHoodManual`, ...) are unrelated and fine.

## Enforcement

A static scanner, `tools/mutex_check.py`, rejects any of the above in one pass:

```bash
python3 tools/mutex_check.py .        # exit 0 = clean, 1 = violations
```

It scans every `.java` under `TeamCode/.../teamcode/`, classifies each file by
package, and reports `file:line` for every violation. It ignores comments.

The same script gates check-ins via GitHub Actions
(`.github/workflows/mutex-guard.yml`): a **push or pull request that introduces
a mutex invalidation fails the build** until fixed. Locally, opencode can also
run it through the `mutex-guard` skill before you commit.

## When you add a subsystem

- Extend `Subsystem<T>` in `decode/subsystem`.
- Implement `onSet(T)` with package-private (no modifier) access — it just
  writes your private state field.
- Keep `currentState`/`targetState`/`power`/... `private` so only `onSet` (via
  `politeSet`/`forceSet`) can mutate it from outside the class.
- Do not touch `politeSet` / `forceSet` / `setLocked`.
- Run `python3 tools/mutex_check.py .` — it must exit 0.
