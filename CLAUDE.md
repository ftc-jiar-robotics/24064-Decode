# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the FTC (FIRST Tech Challenge) robot code for team 24064 (DECODE), 2025-2026 season. It is an Android Gradle project built with Android Studio and deployed to a Control Hub running an FTC Robot Controller app.

## Build & Deploy

This project requires Android Studio Ladybug (2024.2) or later. There is no terminal-based build for running on a robot — code must be deployed via Android Studio:

- **Build:** Use Android Studio's Gradle sync and build system (`./gradlew assembleDebug` for a command-line build)
- **Deploy:** Connect to the Robot Controller via USB or Wi-Fi, then use Android Studio to install the APK
- **No unit tests** are present; all testing is done on physical hardware

## Code Architecture

### Package Structure

All team code lives under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/`:

- **`subsystem/`** — Core robot hardware abstractions
- **`opmodes/`** — TeleOp and Autonomous OpModes that run on the robot
- **`control/`** — Control theory components (PID, filters, state controllers, solvers)
- **`util/`** — Utility classes (ActionScheduler, caching wrappers, sensors)
- **`sensor/`** — Sensor abstractions (color, distance, IMU)

### Subsystem Pattern

Every mechanism extends `Subsystem<T>` (a generic abstract class with a state enum). Key methods:
- `set(T state)` — sets state; blocked if locked
- `set(T state, boolean isOverride)` — bypasses lock when `isOverride=true`
- `run()` — called every loop to execute state machine logic
- `setLocked(boolean)` — prevents state changes during Actions (used by ActionScheduler)

Subsystems: `Shooter`, `Intake`, `GateOpener`, `Flywheel`, `Feeder`, `Hood`, `Turret`

### Robot Class (`subsystem/Robot.java`)

The singleton entry point. `Common.robot` holds the global reference. `Robot.run()` must be called every loop and internally calls `bulkReader.bulkRead()`, then updates drivetrain, shooter, intake, gateOpener, and actionScheduler.

- TeleOp constructor: `new Robot(hardwareMap)` — initializes ArduCam, no Limelight
- Auto constructor: `new Robot(hardwareMap, true)` — initializes Limelight, no ArduCam

### Common (`subsystem/Common.java`)

Central config class annotated with `@Config` (FTC Dashboard). Contains:
- All hardware config name strings (used to map `hardwareMap.get(...)`)
- Global state flags (`isRed`, `isTelemetryOn`, `isSlowMode`, `isFuturePoseOn`, etc.)
- Field geometry constants (goal poses, triangle positions, distances)
- `Common.robot` — global Robot reference set at OpMode init
- `Common.telemetry` / `Common.dashTelemetry` — telemetry singletons

### ActionScheduler (`util/ActionScheduler.java`)

A non-blocking action queue (inspired by QC 21229) for use in TeleOp. Wraps RoadRunner `Action` objects. Actions are automatically bracketed with subsystem lock/unlock (`InstantAction`). Use `addAction()` to queue; `run()` to tick in the loop; `runBlocking()` for auto sequences.

### Auto Architecture

All autos extend `AbstractAuto`, which provides:
- `configure()` — init-loop to select alliance color and calibrate IMU
- `onInit()` — build paths, set follower
- `onRun()` — override to sequence `robot.actionScheduler.addAction(...)` + `robot.actionScheduler.runBlocking()` calls
- Automatically saves end pose to `Common.AUTO_END_POSE` for TeleOp continuity

Auto paths are built in `opmodes/auto/path/GoalPaths.java` and `AudiencePaths.java` using PedroPathing `PathChain` objects. Paths support mirroring for red/blue alliance.

### Shooter Pipeline

`Shooter` (state: IDLE → PREPPING → RUNNING) orchestrates:
1. **Turret** (`ODOM_TRACKING` state) — uses odometry pose + Limelight/ArduCam to auto-aim at the goal using analog servo encoder feedback
2. **Flywheel** (`ARMING` → `RUNNING`) — spins up to target RPM with voltage compensation
3. **Feeder** (`BLOCKING` → `RUNNING`) — releases ball when flywheel is at speed and turret is in tolerance
4. **Hood** — servo angle set by lookup table (distance-based or RPM-based)

Shot sequencing: queue shots with `shooter.incrementQueuedShots(n)` or `setQueuedShots(n)`. The system transitions automatically through IDLE → PREPPING → RUNNING → IDLE.

### Localization

Uses PedroPathing with a Pinpoint localizer (dead wheels). Two re-localization sources:
- **ArduCam** (`relocalizeWithArdu()`) — camera-based pose update in TeleOp, filtered by staleness and variance thresholds
- **Wall snap** (`relocalizeWithWall()`) — hard-sets pose to known wall position

### Control Library (`control/`)

Custom implementations of:
- PID, feedforward, full-state feedback controllers
- IIR low-pass filters, differentiators, integrators
- Gain matrices (HSV, LowPassGains)
- Kinematics solver for turret aiming

### Caching Wrappers (`util/`)

`CachedServo`, `CachedMotor`, `CachedCRServo` — only write hardware if the value changed, reducing I2C/SPI bus traffic. `BulkReader` performs a single bulk read per loop for all hubs.

### FTC Dashboard Integration

`@Configurable` (Bylazar) and `@Config` (FTC Dashboard) annotations expose static fields for live tuning via the dashboard at `192.168.43.1:8080` when connected to the robot's Wi-Fi.

### TeleOp (`opmodes/MainTeleOp.java`)

Single-driver TeleOp. Key behaviors:
- Reads `Common.AUTO_END_POSE` at start to continue from auto end position
- Auto-shoots when `Common.inTriangle == true` (robot is in launch zone) and a ball is detected
- ArduCam re-localizes every loop when stationary
- Gamepad rumble feedback when robot is full of balls
