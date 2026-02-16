# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FRC Team 8041 (Lake City Ultrabots) 2026 competition robot. Java-based, using WPILib's command-based framework with AdvantageKit logging. Runs on a RoboRIO 2.0 with CTRE Phoenix 6 swerve drivetrain and Limelight 4 vision.

## Build Commands

```bash
./gradlew build            # Compile and build
./gradlew deploy           # Deploy to robot (requires robot connection)
./gradlew simulateJava     # Run desktop simulation
./gradlew test             # Run JUnit 5 tests
```

On Windows, use `gradlew.bat` directly if `./gradlew` doesn't work in your shell.

## Architecture

### Lifecycle Flow

`Main.java` → `RobotBase.startRobot(Robot::new)` → `Robot` (extends `LoggedRobot`) → creates `RobotContainer` which wires all subsystems, commands, and controller bindings. `CommandScheduler.run()` is called every robot periodic cycle.

### Subsystem Pattern

Each subsystem lives in its own package under `src/main/java/frc/robot/subsystems/` with a subsystem class and a constants class. Subsystems use **state machine enums** (e.g., `ShooterState`, `IntakeState`) where motor commands are only sent on state transitions (`currentState != lastState`), reducing CAN bus traffic.

### Command Pattern

Commands are created via **static factory classes** (`ShooterCommands`, `IntakeCommands`) that delegate to subsystem methods returning `Command` objects. Vision alignment uses a standalone `Limelight_Move` command. All command bindings are in `RobotContainer.configureBindings()`.

### CAN Bus Layout

- **Default bus (`""`):** Swerve drive motors, CANcoders, Pigeon 2 IMU
- **CANivore (`"Jeffery"`):** Shooter motors (IDs 2, 3), intake motors (IDs 4, 5)

### Vision Integration

`LimelightSubsystem` reads from NetworkTables (`limelight-april`) and has a full simulation mode. Vision poses are fused into the CTRE swerve Kalman filter via dependency-injected pose supplier/consumer set in `RobotContainer`, avoiding circular dependencies. Auto-aim heading lock is embedded directly in the default drive command lambda.

### Autonomous

PathPlanner 2026 with holonomic path following. Named commands registered in `RobotContainer.configureAutoBuilder()`. Auto routines stored in `src/main/deploy/pathplanner/autos/`. Translation PID kP=10, rotation PID kP=7.

### Logging

AdvantageKit writes WPILOG to USB (`/U/logs`) and streams via NT4. CTRE SignalLogger records swerve data to `.hoot` files. Subsystems publish state to SmartDashboard (e.g., `Shooter/State`, `Limelight/Has Target`, `Vision/Status`).

## Key Files

- `RobotContainer.java` — central wiring: subsystem creation, command bindings, auto builder config
- `generated/TunerConstants.java` — CTRE Tuner X auto-generated swerve constants (do not hand-edit)
- `LimelightHelpers.java` — third-party Limelight helper library (do not hand-edit)
- `ShooterConstants.java` — includes distance-to-velocity interpolation table for vision tracking
- `VisionConstants.java` — tag groups, PID gains, alignment tolerances

## Vendor Libraries

Phoenix6 26.1.0 (CTRE hardware), PathplannerLib 2026.1.2, AdvantageKit 26.0.0, ChoreoLib 2026.0.1 (installed but unused). Descriptors in `vendordeps/`.

## Known Issues

- Climber subsystem is stubbed out (empty files, commented out in RobotContainer)
- `IntakeSubsystems.setVelocity()` applies the same velocity to both intake and pivot motors — these should be independent
- Intake PID gains are all `0.1` placeholders needing tuning
- `VisionConstants` camera mounting values (height 12in, angle 45°) are placeholders
- `Logger.recordMetadata("ProjectName", "Elevator Simulation")` in Robot.java is stale
