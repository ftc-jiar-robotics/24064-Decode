# AGENTS.md

FTC 2025-2026 robot code for team 24064 (DECODE). Android Gradle app deployed to a Control Hub. Read `CLAUDE.md` for the full architecture guide — this file only adds what is not obvious from filenames.

## Build & verify

- **No unit tests exist.** Verification = successful compile. The only meaningful check is `./gradlew assembleDebug` from the repo root.
- Deploying to the robot requires Android Studio (USB/Wi-Fi); there is no CLI deploy.
- Debug builds auto-sign with `libs/ftc.debug.keystore` (alias/password `android`) — no signing setup needed for local builds.
- Requires Android SDK; `local.properties` (gitignored, machine-specific) must point `sdk.dir` at an installed SDK. Android Studio Ladybug+ / AGP 8.13 / Gradle 8.13.
- Note: `org.gradle.jvmargs=-Xmx1024M` — large builds may need more heap; bump in `gradle.properties` if OOM.

## Module layout

- `TeamCode/` — **only module you should edit.** Team code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/decode/`.
- `FtcRobotController/` and `PedroPathingVisualizer/` — vendored FTC SDK / upstream modules. Do not edit.
- `build.common.gradle` and `build.dependencies.gradle` — shared FTC SDK build files; avoid editing (SDK updates overwrite them). Dependency versions live in `build.dependencies.gradle`.
- Root `README.md` is the upstream FTC SDK README (~130KB), not team docs. `CLAUDE.md` is the team doc.
- `trend_analysis*.py` at root are offline data-analysis scripts, unrelated to the robot build.

## Repo-specific gotchas

- PedroPathing tuning constants live at `TeamCode/.../teamcode/pedroPathing/Constants.java` and `Tuning.java` — **outside** the `decode/` package, but they are team-edited (motor names, PIDF gains, localizer offsets).
- `leboofy.xml` (repo root) is the robot hardware configuration. Hardware device names there must match the config-name strings in `decode/subsystem/Common.java`.
- Many classes are annotated `@Config` (FTC Dashboard) / `@Configurable` (Bylazar): static fields are live-tunable at `192.168.43.1:8080` on the robot's Wi-Fi. Never hardcode a value that already has a `@Config` field.
- `Common.java` is the central config: hardware names, field geometry, global flags (`isRed`, `isTelemetryOn`, `inTriangle`, ...). Most tuning happens here, not in the consuming classes.
- `Robot.java` (in `decode.robot`, NOT the subsystem package) has two constructors that differ in camera setup: `new Robot(hardwareMap)` (TeleOp → ArduCam, no Limelight) vs `new Robot(hardwareMap, true)` (auto → Limelight, no ArduCam). Use the right one for the OpMode.
- `Robot.run()` must be called every loop — it drives `bulkReader.bulkRead()` and every subsystem. Do not add per-loop hardware reads outside subsystems.
- All mechanisms extend `Subsystem<T>`; state changes are blocked while locked (`ActionScheduler` locks subsystems around actions). OpModes use `politeSet(T)` (returns false while locked); only `decode.subsystem` code (actions) may use `forceSet(T)`. `Robot.java` is outside the boundary — it must never call `forceSet`/`setLocked`. Subsystems must not expose public mutators; exceptions are listed in `tools/mutex_whitelist.txt`. See `MUTEX.md`.
- **Run `python3 tools/mutex_check.py` after editing subsystems/opmodes and before committing** — it guards the mutex privilege boundary and is enforced by the `mutex-guard` GitHub Actions workflow (push/PR).

## Workflow

- Work happens on feature branches (`feature/*`, `release/*`, loose names like `shooting-kinematics`); `main` is the stable baseline. Commit messages are informal and tuning-oriented.
- Autos are sequenced in `opmodes/auto/Auto*.java` via `robot.actionScheduler.addAction(...)` + `runBlocking()`; paths are built in `opmodes/auto/path/GoalPaths.java` and `AudiencePaths.java` with red/blue mirroring.
