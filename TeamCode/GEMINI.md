# Decode 24064 FTC Robot Code

This repository contains the robot controller code for FTC Team 24064 (Decode). The project is built on the FTC Robot Controller SDK and follows a modular, subsystem-based architecture.

## Project Overview

- **Core Framework**: Custom subsystem-based architecture where each hardware component (Shooter, Intake, Turret, etc.) inherits from a base `Subsystem` class.
- **Navigation**: Uses [Pedro Pathing](https://github.com/pedropathing/pedro-pathing) for advanced autonomous trajectory following and localization.
- **Vision**: Integrates Limelight 3A for autonomous targeting and ArduCam for real-time relocalization during TeleOp.
- **Control Systems**: Implements advanced kinematics (see `KinematicsSolver.java`) for turret and shooter positioning.
- **Main Entry Points**:
    - **TeleOp**: `org.firstinspires.ftc.teamcode.decode.opmodes.MainTeleOp`
    - **Autonomous**: Located in `org.firstinspires.ftc.teamcode.decode.opmodes.auto`
    - **Hardware Hub**: `org.firstinspires.ftc.teamcode.decode.subsystem.Robot`

## Directory Structure

- `src/main/java/org/firstinspires/ftc/teamcode/decode/`:
    - `control/`: Control algorithms, PID gains, and matrix calculations.
    - `opmodes/`: Robot programs (TeleOp, Auto, and Prototypes).
    - `sensor/`: Wrappers for sensors like color sensors and vision systems.
    - `subsystem/`: Hardware abstraction layers (Drivetrain, Shooter, Intake, etc.).
    - `util/`: Utility classes for logging, bulk reading, and action scheduling.
- `src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/`: Configuration and tuning for the Pedro Pathing library.

## Building and Running

Since this is an Android-based project, it is typically managed through Android Studio.

### Prerequisites
- Android Studio with the latest Android SDK.
- FTC Robot Controller App dependencies (handled via Gradle).

### Commands
- **Build**: `./gradlew :TeamCode:assembleDebug`
- **Install to Robot Controller**: `./gradlew :TeamCode:installDebug`
- **Clean**: `./gradlew clean`

## Development Conventions

- **Subsystem Pattern**: All new hardware modules should extend `Subsystem<T>` and implement `run()`, `get()`, `set()`, and `printTelemetry()`.
- **Initialization**: Hardware mapping and subsystem instantiation must be performed in the `Robot` class constructor.
- **Non-blocking Code**: Avoid using `sleep()` in OpModes. Use `ActionScheduler` or state machines within subsystems to handle timed events.
- **Telemetry**: Use `printTelemetry()` within subsystems to report data to the Driver Station and FTC Dashboard.
- **Constants**: Configuration constants (PID gains, offsets, etc.) are often annotated with `@Configurable` for real-time tuning via FTC Dashboard.

## Key Files
- `Robot.java`: The central coordinator for all hardware and subsystems.
- `MainTeleOp.java`: The primary driver-controlled OpMode.
- `KinematicsSolver.java`: Handles complex positioning logic for the shooter system.
- `Constants.java` (in `pedroPathing`): Drivetrain and follower configuration.
