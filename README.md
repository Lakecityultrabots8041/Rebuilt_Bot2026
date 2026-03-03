# Team 8041 - Lake City Ultrabots | 2026 REBUILT

FRC Team 8041's competition robot for the 2026 REBUILT season.

Java, WPILib command-based framework, AdvantageKit logging, Phoenix 6 swerve drivetrain.

## Quick Start

1. Clone this repo
2. Open in WPILib VS Code
3. `./gradlew build` to compile (use `gradlew.bat` on Windows if needed)
4. `./gradlew deploy` to push code to the RoboRIO
5. `./gradlew simulateJava` to run in simulation

## Hardware Overview

| Component  | Details                                                                               |
|------------|---------------------------------------------------------------------------------------|
| Controller | RoboRIO 2.0                                                                           |
| Drivetrain | CTRE Phoenix 6 swerve (4 TalonFX drive + 4 TalonFX steer + 4 CANcoders)               |
| IMU        | Pigeon 2                                                                              |
| Vision     | 2x Limelight 4 (`limelight-april` on shooter side, `limelight-intake` on intake side) |
| LEDs       | REV Blinkin on PWM port 0                                                             |
| CAN Buses  | Two buses, one for swerve and one for intake/shooter                                  |

### Motor Map

| Motor                       | CAN ID | Use                                              |
|-----------------------------|--------|--------------------------------------------------|
| Intake pivot                | 2      | MotionMagic position control with gravity comp   |
| Intake roller               | 3      | Duty cycle intake/eject                          |
| Shooter floor feed          | 5      | Feeds ball into flywheel                         |
| Shooter flywheel 1          | 6      | Velocity PID for shooting                        |
| Shooter ceiling feed        | 7      | Feeds ball into flywheel                         |
| Shooter upper feed (Lo4d3r) | 8      | 12:1 gearbox, 4 belts                            |

## Subsystems

### Swerve Drivetrain
Phoenix 6 swerve with CTRE Tuner X generated constants. Field-centric drive with asymmetric slew rate limiting (fast acceleration, gentler deceleration to prevent tip-overs). Both Limelights fuse MegaTag2 pose estimates into the swerve Kalman filter each loop. See [Drive Tuning](docs/DRIVE_TUNING.md) for PID and slew tuning.

### Shooter
Two independent state machines (flywheel + feed rollers) that only send motor commands on state changes to reduce CAN traffic. The flywheel runs velocity PID and supports a vision tracking mode that adjusts speed based on distance to the hub. See [Shooter Tuning](docs/SHOOTER_TUNING.md) for PID and distance table tuning.

### Intake
Pivot arm with MotionMagic profiling and cosine gravity compensation. Three positions: stow, travel, and intake (fully deployed). Roller runs at full power for intake/eject. See [Intake Tuning](docs/INTAKE_TUNING.md) for MotionMagic and position tuning.

### Vision (Limelight)
Two Limelight 4 cameras, each in their own `LimelightSubsystem` instance. Handles AprilTag detection, distance calculation, and pose fusion. Full simulation support with fake tag detection from field layout. Powers the auto-aim and vision alignment commands. See [Vision System](docs/VISION.md) for the full breakdown.

### LEDs
REV Blinkin controller with a priority-based state waterfall: vision aligned (green) > auto-aim active (yellow) > intaking (orange) > autonomous (rainbow) > disabled (alliance breath) > idle (fire effect).

### Climber
Stubbed out, not yet implemented.

## Controls (Xbox Controller, Port 0)

| Input                | Action                                |
|----------------------|---------------------------------------|
| Left Stick           | Field-centric drive                   |
| Right Stick X        | Rotation (when auto-aim is off)       |
| Right Bumper         | Reset field-centric heading           |
| Left Bumper          | Toggle auto-aim (PID lock to hub tags)|
| Right Trigger (hold) | Shoot (flywheel + all feeds)          |
| Left Trigger (hold)  | Eject (reverse feeds)                 |
| B (hold)             | Pass                                  |
| X (hold)             | Intake roller on                      |
| A                    | Pivot to travel                       |
| D-Pad Up             | Pivot to stow                         |
| D-Pad Down           | Pivot to intake                       |
| D-Pad Left (hold)    | FollowTag demo mode (outreach only)   |
| START (hold)         | Vision align to hub                   |
| BACK (hold)          | Vision align to tower                 |
| Y (hold)             | Vision align to outpost               |

## Autonomous

PathPlanner holonomic path following. Auto chooser published to SmartDashboard as "Auton Mode". Default auto is `SimplePath Auton`.

### Naming Convention
- **Captain** = immediate start (alliance captain, no wait)
- **1st Pick** = 5-second wait at start
- **L / M / R / T** = starting position (Left, Mid, Right, Trench)
- **W** = Wait, **G** = Go/drive, **T** = Trench, **L** = Load from outpost, **D** = Depot, **S** = Shoot

### Available Autos

**Captain (no wait):**
- Captain L GDS, Captain L GTLS
- Captain M GDS, Captain M GTLS
- Captain R GDS, Captain R GTLS
- Captain T GTLS

**1st Pick (5s wait):**
- 1st Pick L WGS, 1st Pick L WGTLS
- 1st Pick M WGS, 1st Pick M WGTLS, 1st Pick M WGLTDS
- 1st Pick R WGS, 1st Pick R WGTLS
- 1st Pick T WGTLS

**Standalone/Test:**
- SimplePath Auton (default, drive only)
- LeftPathShoot
- TestAutoCenter

## Project Structure

```
src/main/java/frc/robot/
    Robot.java              - Minimal LoggedRobot, shift tracking
    RobotContainer.java     - All subsystems, bindings, auto config
    subsystems/
        drive/              - CommandSwerveDrivetrain, DriveConstants
        shoot/              - ShooterSubsystem, ShooterConstants
        intake/             - IntakeSubsystems, IntakeConstants
        vision/             - LimelightSubsystem, VisionConstants
        led/                - LEDSubsystem
        climb/              - ClimberSubsystem (stubbed)
    commands/
        ShooterCommands.java    - Static factory for shooter sequences
        IntakeCommands.java     - Static factory for intake sequences
        Limelight_Move.java     - Vision alignment command (hub/tower/outpost/trench)
        FollowTag_Demo.java     - Demo mode tag following
    generated/
        TunerConstants.java     - CTRE Tuner X generated (do not hand-edit)

src/main/deploy/pathplanner/
    autos/                  - PathPlanner auto routines
    paths/                  - PathPlanner paths
```

## Documentation

- [Vision System](docs/VISION.md)
- [Controller Map](docs/CONTROLLER_MAP.md)
- [Drive Tuning](docs/DRIVE_TUNING.md)
- [Intake Tuning](docs/INTAKE_TUNING.md)
- [Shooter Tuning](docs/SHOOTER_TUNING.md)
- [Autonomous Strategy](docs/AUTONOMOSE.md)

## Known Issues

- Climber subsystem is stubbed out (not implemented)
- Camera mounting values (height, angle) are placeholders, need to be measured on robot
- Flywheel stator current limits at Phoenix 6 defaults (120A), should be lowered to ~80A
- Intake pivot hardware limits and soft limits are disabled (see FIXME in IntakeSubsystems.java)
- Steer motor kP may be too high (ratcheting sound), check TunerConstants.java

## Vendor Libraries

- CTRE Phoenix 6 (26.1.0)
- PathPlannerLib (2026.1.2)
- AdvantageKit
- ChoreoLib (2026)
