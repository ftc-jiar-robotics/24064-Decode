# Onboarding — DECODE (24064) Code Architecture

A short map of how the robot code is organized, who is allowed to set a
subsystem's state, and the mutex rules you must follow. New developers: read
this first, then `MUTEX.md` for the full contract and `AGENTS.md` for repo
gotchas.

Before the architecture, understand the one idea everything is built around.

## The mutex — what it is and why it exists

### What is it?

Every `Subsystem<T>` carries a private lock. `ActionScheduler.addAction(...)`
wraps each action in three steps:

1. `setLocked(true)`  → the subsystems the action touches are **locked**;
2. run the action;    → actions can `forceSet` freely;
3. `setLocked(false)` → released.

While locked, `politeSet(T)` is a no-op that returns `false` — it refuses to
touch the state. `forceSet(T)` ignores the lock (that's why it's
package-private).

### Why do we have it?

An action is a **choreographed sequence**. Example: `shootArtifacts(3)`
ramps the intake, queues shots, arms the flywheel, PID-aims the turret, waits
for the ball, fires, clears the queue, and restores drive speed — in that
order. Each step assumes the previous step's state is still in place.

If anything else wrote to those subsystems mid-sequence — a stray gamepad
trigger, an OpMode loop that flips the intake off, a manual hood angle yanked
in while the turret is tracking — the choreography desynchronizes:

- the shooter fires at the wrong time or angle,
- the intake fights the feeder for the ball,
- a motor stalls against a locked mechanism.

That's not just a lost match: stalled motors and fighting mechanisms are how
you **strip gears and physically break the robot**. The mutex serializes
control — while an action owns a subsystem, everyone else's request is
politely ignored. The action's sequence cannot be corrupted from outside.

Everything below — the package layout, the diagram, and the rules — exists to
make that guarantee hold.

---

## Where things live

All team code is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/`:

| Package                  | Contents                                                              |
|--------------------------|-----------------------------------------------------------------------|
| `decode.robot`           | `Robot.java` — the composition root. Owns every subsystem.            |
| `decode.subsystem`       | `Subsystem<T>` base + every mechanism + `RobotActions` + `ActionScheduler`. **This package owns the mutex.** |
| `decode.opmodes`         | `MainTeleOp` (tele-op) and `auto/Auto*.java` (autonomous).            |
| `decode.util` / `sensor` / `control` | Support code (bulk reads, cameras, vision, PID/kinematics). |

---

## The block diagram

Four players: **OpModes**, **Robot**, **RobotActions**, and **Subsystems**.
The numbered arrows show the *only* legal ways to change subsystem state.

![Code structure drawing](JankProgramming.png)

The same structure, as ASCII art:

```
 (5) politeSet(T): an OpMode can drive a                  ┌──────────────────────────────────────────┐
     subsystem directly. It is lock-aware:                │                 OPMODES                  │
     it no-ops (returns false) while an                   │         MainTeleOp · Auto*.java          │
     Action holds the lock. Example:                      └────────┬───────────────────────┬─────────┘
     robot.intake.politeSet(trigger1);                             │                       │
                    │         (1) creates + owns                   │                       │           (3) queues a sequence:
                    │             Robot, calls run()               │                       │               robot.actionScheduler
                    │             every loop                       │                       │               .addAction(...)
                    │                                              ▼                       ▼
                                                          ┌──────────────────┐
                    │                                     │      ROBOT       │
                    │                                     │   decode.robot   │
                    │                                     │ composition root │
                    │                                     │ owns subsystems  │
                                                          └────────┬─────────┘
                                                                                  ┌──────────────────┐
                    │                                                             │   ROBOTACTIONS   │
                    │                                                             │  static Action   │
                    │                                                             │    factories     │
                    │                                                             │  subsystem pkg   │
                                                                                  └────────┬─────────┘
                    │         (2) robot.run() drives               │                       │           (4) forceSet(T)
                    │             every subsystem's run()          │                       │               bypasses the lock
                    │                                              ▼                       ▼
                    ▼
                    ┌────────────────────────────────────────────────────────────────────────────────┐
                    │                                   SUBSYSTEMS                                   │
                    │                         Intake · GateOpener · ParkLift                         │
                    │                      Shooter (composite): Hood · Flywheel                      │
                    │                                         Turret · Feeder                        │
                    └────────────────────────────────────────────────────────────────────────────────┘
```

### What each arrow means

| # | Path                              | Function used     | Notes                                                                 |
|---|-----------------------------------|-------------------|-----------------------------------------------------------------------|
| 1 | **OpMode → Robot**                | `new Robot(...)`  | Tele-op: `new Robot(hardwareMap)`. Auto: `new Robot(hardwareMap, true)` (uses Limelight instead of ArduCam). |
| 2 | **Robot → Subsystems**            | `run()`           | `Robot.run()` must be called every loop. It never `forceSet`s — it just runs each subsystem's `run()` and its scheduler. Robot may only *request* state via `politeSet` (or a whitelisted configuration setter, see rule 7). |
| 3 | **OpMode → RobotActions**         | `addAction(...)`  | `robot.actionScheduler.addAction(RobotActions.shootArtifacts(3))`. The scheduler wraps the action in `lock → action → unlock`. Autos then call `runBlocking()` to pump the queue. |
| 4 | **RobotActions → Subsystems**     | `forceSet(T)`     | Actions live in `decode.subsystem`, so they may force state even while locked. This is the *only* caller category allowed to. |
| 5 | **OpMode → Subsystems (direct)**  | `politeSet(T)`    | Tele-op only. Lock-respecting: returns `false` and changes nothing while an Action owns the subsystem. |

**The takeaway:** if you are outside `decode.subsystem`, the *only* way to
change a subsystem's state is `politeSet(T)` (and read-only `getXxx()`). If you
need to run a *sequence* of state changes, that's what a `RobotActions` action
is for — it can `forceSet`, and the scheduler holds the mutex for the whole
sequence.

---

## Setting nested subsystems (composites)

`Shooter` is a **composite**: it doesn't contain just one mechanism — it owns
child subsystems (`Hood`, `Flywheel`, `Turret`, `Feeder`, `KinematicsSolver`).
The parent drives its children from inside `run()`:

```
    ┌──────────────────────────────────────────────────────────────┐
    │                      Shooter  (parent)                       │
    │                                                              │
    │  In run(), the parent drives its children. Same package,     │
    │  so it uses forceSet(T) -- NEVER politeSet: while an Action  │
    │  holds the lock, the children are locked too, so politeSet   │
    │  would silently return false and break the sequence.         │
    │                                                              │
    │      flywheel.forceSet(FlyWheelStates.RUNNING);              │
    │      feeder.forceSet(Feeder.FeederStates.RUNNING);           │
    │      hood.forceSet(hood.getHoodAngleWithDistance(distance)); │
    │      turret.forceSet(Turret.TurretStates.ODOM_TRACKING);     │
    │                                                              │
    │  Shooter.setLocked(...) fans the lock out to every child so  │
    │  the whole assembly is locked / unlocked together:           │
    │      super.setLocked(locked);                                │
    │      feeder.setLocked(locked);  flywheel.setLocked(locked);  │
    │      turret.setLocked(locked);  hood.setLocked(locked);      │
    │                                                              │
    │    ┌───────────┐  ┌───────────┐  ┌───────────┐  ┌───────────┐ │
    │    │   Hood    │  │  Flywheel │  │   Turret  │  │   Feeder  │ │
    │    └───────────┘  └───────────┘  └───────────┘  └───────────┘ │
    └──────────────────────────────────────────────────────────────┘
```

**Internal rule:** parent → child always uses `forceSet(T)`. Outside code never
`forceSet`s a child either — it goes through the parent's package-private
helper (e.g. `robot.shooter.setFeederIdle(...)`, which internally calls
`feeder.forceSet(...)`).

---

## The custom actions in `Actions.java`

The action utilities live in `decode/util/Actions.java`. They wrap the base
RoadRunner `Action` interface to give you the wait / skip / timeout behavior an
action needs:

| Action | Constructor | What it does |
|--------|-------------|--------------|
| `RunnableAction` | `(Callable<Boolean> action)` | Runs the callable every loop; the action keeps running while it returns `true` and finishes when it returns `false`. Ad-hoc work or conditions. |
| `SingleCheckAction` | `(Callable<Boolean> check, Action action)` | Evaluates `check` once: if `true`, runs `action` to completion; if `false`, finishes instantly (skip). The guard + body primitive — "skip if already done." |
| `UntilConditionAction` | `(Callable<Boolean> check, Action action)` | Runs `action` while `check` is `false`; stops as soon as `check` is `true` (and breaks path-following if the action is a `FollowPathAction`). "Run until …" |
| `TimedAction` | `(Action action, long maxTimeMs, String name)` | Wraps an action so it stops after `maxTimeMs` even if it hasn't finished — a time-budgeted action. |
| `CallbackAction` | `(Action action, PathChain chain, double startCondition, int index, Follower f, String s)` | Waits until a PedroPathing callback at a given index / position along a path fires, then runs `action`. Defer work until the robot reaches a point on a path. |

---

## How to write a RobotAction

### The one rule

**Never assume where the robot is when an action starts.** An action must be
written to bring the robot from *any* state into a *known* state. If part of
the goal is already achieved, that part must skip itself.

The scheduler can run your action at any moment: mid-drive, right after another
action, immediately after tele-op starts. If `score()` assumed "the sample is
already transferred to the claw," then a score called before any transfer would
feed nothing and break the cycle. Actions defend against that by checking
their own prerequisites.

### Guard + body

Every action is a **guard** followed by the **body**. If the guard is true the
body runs to completion; if it's false the action finishes instantly — nothing
happens. That is exactly what `Actions.SingleCheckAction` does:

```java
public static Action transferToClaw() {
    return new Actions.SingleCheckAction(
            () -> robot.currentState != Robot.State.TRANSFERRED,  // guard: still needs doing?
            new SequentialAction(                                  // body
                    retractForTransfer(),
                    setWrist(Arm.WristAngle.COLLECTING, WRIST_COLLECTING_TRANSFER_TO_CLAW),
                    setArm(Arm.ArmAngle.COLLECTING, ARM_COLLECTING_TRANSFER_TO_CLAW),
                    // ... the rest of the transfer sequence ...
                    new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
            )
    );
}
```

Write the guard as *"this still needs doing"*: if the robot is already in
`TRANSFERRED`, the guard is false and the whole sequence is skipped. Two
semantics to keep straight:

- `SingleCheckAction` does **not wait** for the guard to become true — it
  evaluates it and either runs the body or skips. Use it for "skip if
  already done / skip if the precondition isn't met."

### Compose big actions out of guarded small ones

A big action is just a `SequentialAction` of smaller guarded actions. The
"scoring requires transfer first" dependency falls out for free:

```java
public static Action scoreBasket() {
    return new Actions.SingleCheckAction(
            () -> robot.currentState != Robot.State.SCORED_SAMPLE,
            new SequentialAction(
                    transferToClaw(),        // skips itself if already transferred
                    setupScoreBasket(true),  // skips itself if already set up
                    dropSample()             // skips itself if already scored
            )
    );
}
```

```
  scoreBasket()
    │  guard: already SCORED_SAMPLE? ── yes ──▶ skip (finishes instantly)
    ▼
    ├─ transferToClaw()        guard: already TRANSFERRED?  ── yes ──▶ skip
    ├─ setupScoreBasket(true)  guard: already SETUP_SCORE?   ── yes ──▶ skip
    └─ dropSample()            guard: already NEUTRAL?       ── yes ──▶ skip
```

Each prerequisite is itself an action with its own guard, so composition is
safe from any starting state and any call order. This is exactly how the
previous season's `RobotActions`
(`24064-IntoTheDeep`) was built — `transferToClaw()` calls
`retractForTransfer()` first, which skips itself once the robot is already
transfer-ready.

### Declare the end state

Every action finishes by recording the state it has brought the robot to, so
the *next* action's guard can decide to skip:

```java
new InstantAction(() -> robot.currentState = Robot.State.TRANSFERRED)
```

The end state must be specific enough to reason about: "transferred",
"setup for basket", "scored" — never a vague "done".

### In this repo

`RobotActions` (in `decode.subsystem`) already uses the same
`Actions.SingleCheckAction` utility. This season's subsystems expose `get()`
instead of a global `robot.currentState`, so guards read the subsystem's
commanded state:

```java
public static Action openGate() {
    return new Actions.SingleCheckAction(
            () -> robot.gateOpener.get() != GateOpener.GateOpenerStates.OPEN,
            new InstantAction(() -> robot.gateOpener.forceSet(GateOpener.GateOpenerStates.OPEN))
    );
}
```

Rules of thumb:

- Guard on **commanded** state (`get()` / target value), not a raw sensor
  reading — commanded state is deterministic and won't flicker.
- If the guard is "still needs doing," the end-state write must be the *last*
  step of the body.
- Put independent steps in a `ParallelAction`; keep dependent steps in
  `SequentialAction` (e.g. always retract before transfer).
- Never write an action that assumes the robot started in the state it ends
  in.

There are 2 ways of writing actions: 

1. Directly control the movments of State Machine-less Subsystems and set their states(most commonly useful in Pick and Place games)
2. Set stimulus that Subsystems observe and automatically move through their State Machine to fulfill

---

## The rules

1. **OpModes use `politeSet(T)` only.** It returns `false` while locked — treat
   that as "an action is in control, my request is ignored." Never call
   `forceSet` from an OpMode; it is not even visible there.
2. **Only `decode.subsystem` code may `forceSet`.** That's `RobotActions`,
   `ActionScheduler`, and composite parents (`Shooter`) driving children.
3. **Never widen the mutex API.** `forceSet`, `setLocked`, and the
   `onSet_DONOTCALL` hook must stay package-private. A `public` override leaks
   the lock bypass to OpModes.
4. **Never re-declare `politeSet` / `forceSet`.** They are `final` in
   `Subsystem.java`; a subclass re-declaring them breaks base-class
   enforcement.
5. **The `onSet_DONOTCALL` hook is base-class-only.** Subclasses *implement* it
   (package-private, writes only their own private state field); nothing ever
   *calls* it on an instance.
6. **`setLocked` is scheduler-only.** Only the action scheduler path acquires
   or releases the lock. A composite may override it *package-privately* to
   fan the lock out to its children.
7. **`Robot.java` is outside the boundary.** It lives in `decode.robot` (its
   own package) and must never call `forceSet` / `setLocked`. Its only state
   requests are `politeSet` or a **whitelisted, lock-aware configuration
   setter** (e.g. `shooter.setFlywheelMovingToFarZone(...)`), and it drives
   everything through `run()`.
8. **No public mutators on subsystems.** `setXxx` / `clearXxx` / `resetXxx` /
   `toggleXxx` / `applyXxx` / `enableXxx` / `disableXxx` / `incrementXxx` /
   `decrementXxx` methods that change state are forbidden unless listed in
   `tools/mutex_whitelist.txt` — and whitelisted entries must be fail-safe
   (lock-aware) or configuration-type, never state-changing. New functionality
   belongs package-private, behind a `RobotActions` action.
9. **Keep state fields `private`.** Only `onSet_DONOTCALL` (reached via
   `politeSet` / `forceSet`) may write them from outside the class.
10. **`set(...)` is gone.** The old no-mutex `set` API was removed. Use
    `politeSet` (callers) or `forceSet` (subsystem package).

### Which set function, when

| Who is changing state            | Use                     | Result                                      |
|----------------------------------|-------------------------|---------------------------------------------|
| Tele-op OpMode                   | `politeSet(T)`          | Applies, or `false` if locked               |
| Action in `RobotActions`         | `forceSet(T)`           | Always applies (action holds the lock)      |
| Composite parent → child         | `forceSet(T)`           | Always applies (same package)               |
| `Robot`                          | `run()` + `politeSet`   | Never forces; drives every subsystem        |

---

## Verifying your changes

Before committing, run the static gate — it scans every file, classifies it by
package, and reports any mutex invalidation:

```bash
python3 tools/mutex_check.py .        # exit 0 = clean, 1 = violations
python3 tools/mutex_skill_check.py .  # checks docs/skill match the contract
```

The same check runs on GitHub Actions (`.github/workflows/mutex-guard.yml`) on
every push and PR, so a violation fails the build. Full contract and
enforcement details: `MUTEX.md`.

You must also have opencode do the review for you before committing. Launch
opencode in the repo root and invoke the mutex-guard skill:

```bash
cd /Users/personal/Documents/GitHub/24064-Decode
opencode
```

Then prompt it to review your changes, e.g.:

> Run the mutex-guard skill and verify the Subsystem mutex system is intact
> across my edits — check that Robot stays outside the boundary, no public
> mutators were added, and nothing in the subsystem package bypasses the lock.

The skill reads the contract docs and source, reasons over the rules directly
(no regex guesswork), and reports any invalidation before you push.

Then after 
