# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

===============================================================================
SEASON LOCK — NON-NEGOTIABLE
===============================================================================
Claude must use Opus for large code creation.

This robot is built for:

FRC 2026  
Game: REBUILT  

All strategic, scoring, field, and rules-based reasoning MUST apply strictly to the 2026 REBUILT game.

Claude must NOT:
- Assume rules from prior seasons
- Reuse previous game mechanics
- Invent scoring values
- Invent field dimensions
- Invent endgame conditions
- Make variables to formal, kids need to understand how this works

If rules or scoring are unclear:
- Explicitly state uncertainty
- Request official documentation
- Do NOT fabricate

Required uncertainty response:
"I cannot verify this from official 2026 REBUILT documentation."

No invented rules. No speculative geometry.

===============================================================================
PROJECT OVERVIEW
===============================================================================

FRC Team 8041 (Lake City Ultrabots) 2026 competition robot.

Java-based  
WPILib command-based framework  
AdvantageKit logging  

Hardware:
- RoboRIO 2.0
- CTRE Phoenix 6 swerve drivetrain
- Pigeon 2 IMU
- CANcoders
- Limelight 4 vision

===============================================================================
ENGINEERING PRIORITIES (STRICT ORDER)
===============================================================================

1. Robot safety
2. Deterministic behavior
3. Competition reliability
4. CAN bus efficiency
5. Simulation compatibility
6. Student readability
7. Maintainability during build season
8. Elegance (last)

Simplicity beats cleverness.

Do NOT over-architect.

Reliability > abstraction.
Determinism > fancy patterns.

===============================================================================
ANTI-FABRICATION & SOURCE HIERARCHY (STRICT)
===============================================================================

When generating solutions:

1. Check existing repository structure first.
2. Use official WPILib documentation.
3. Use official 2026 REBUILT Game Manual.
4. Use CTRE Phoenix 6 documentation.
5. Use vendor documentation.
6. Then reference reputable FRC GitHub repositories.

Never rely on memory of prior seasons.

Never invent:
- WPILib APIs
- CTRE APIs
- Game rules
- Hardware behavior
- PathPlanner methods

If unsure, ask.

===============================================================================
BUILD COMMANDS
===============================================================================

./gradlew build  
./gradlew deploy  
./gradlew simulateJava  
./gradlew test  

On Windows use gradlew.bat if needed.

===============================================================================
ARCHITECTURE
===============================================================================

Lifecycle Flow:

Main.java  
→ RobotBase.startRobot(Robot::new)  
→ Robot (extends LoggedRobot)  
→ RobotContainer  

CommandScheduler.run() executes every periodic cycle.

Never bypass scheduler structure.

Robot.java must remain minimal.  
RobotContainer wires all subsystems and bindings.

===============================================================================
SUBSYSTEM PATTERN
===============================================================================

Location:
src/main/java/frc/robot/subsystems/

Each subsystem contains:
- Subsystem class
- Folder-scoped constants class

Subsystems use state machine enums:
Example: ShooterState, IntakeState

Motor commands are sent ONLY on state transitions:

currentState != lastState

Purpose:
- Reduce CAN traffic
- Avoid redundant frames
- Improve determinism

Do not remove this optimization without justification.

===============================================================================
COMMAND PATTERN
===============================================================================

Commands created via static factory classes:
- ShooterCommands
- IntakeCommands

Subsystem methods return Command objects.

All bindings belong in:
RobotContainer.configureBindings()

Do not bind commands elsewhere.

Avoid embedding subsystem logic inside autos.

===============================================================================
CONSTANTS STRUCTURE
===============================================================================

Constants are folder-scoped per subsystem.

Example:

subsystems/
    drive/
        DriveConstants.java
    shooter/
        ShooterConstants.java

Constants.java (root) is ONLY for:
- Controller ports
- Rare shared values

No magic numbers anywhere.

Student readability is mandatory.

Do NOT use em dashes (—) in any documentation or comments.
Use commas or end the sentence.

===============================================================================
CAN BUS LAYOUT
===============================================================================

Default bus (""):
- Swerve drive motors
- CANcoders
- Pigeon 2

CANivore ("Jeffery"):
- Intake pivot motor (ID 2)
- Intake roller motor (ID 3)
- Shooter feed floor roller (ID 5)
- Shooter flywheel (ID 6)
- Shooter feed ceiling roller (ID 7)

When configuring motors:
- Confirm motor type
- Confirm bus
- Confirm device ID
- Restore factory defaults
- Set current limits
- Set neutral mode explicitly
- Set inversion explicitly
- Validate firmware compatibility

Never assume defaults.

===============================================================================
SWERVE SYSTEM
===============================================================================

Phoenix 6 swerve  
CTRE Tuner X generated constants:
generated/TunerConstants.java

DO NOT hand-edit TunerConstants.java.

Use proper CTRE kinematics and odometry.

No manual gyro math hacks.

Maintain clean separation:
- Module logic
- Drivetrain subsystem

Pose estimation required.

===============================================================================
VISION INTEGRATION
===============================================================================

Limelight 4  
NetworkTables key: "limelight-april"

Vision subsystem:
- Encapsulates ALL NetworkTables access
- Supports full simulation
- Injects pose supplier/consumer via RobotContainer

Vision fused into CTRE swerve Kalman filter.

Never:
- Access NetworkTables outside vision subsystem
- Hard reset odometry mid-match unless intentional
- Trust single-frame measurement blindly

If using AprilTags:
- Validate ambiguity
- Validate latency
- Reject poor measurements
- Fuse with timestamp

===============================================================================
AUTONOMOUS
===============================================================================

PathPlanner 2026  
Holonomic path following

NamedCommands registered in:
RobotContainer.configureAutoBuilder()

Autos stored in:
src/main/deploy/pathplanner/autos/

Translation PID kP = 10  
Rotation PID kP = 7  

Autos must:
- Be deterministic
- Validate starting pose
- Handle missing vision gracefully
- Not assume perfect localization

===============================================================================
LOGGING
===============================================================================

AdvantageKit:
- WPILOG → /U/logs
- NT4 streaming

CTRE SignalLogger:
- .hoot files

SmartDashboard publishes:
- Shooter/State
- Limelight/Has Target
- Vision/Status

Avoid console spam during matches.
No blocking logs in teleop.

===============================================================================
CONTROL PHILOSOPHY
===============================================================================

PID is fragile.

Use only when necessary:
- Shooter velocity
- Precise positioning

Avoid PID for:
- Intake rollers
- Simple mechanisms

If PID is used:
- Explain each gain
- Keep names readable
- Provide tuning notes

Gravity-loaded mechanisms:
- Motion Magic REQUIRED
- Brake mode REQUIRED
- Current limits REQUIRED
- Soft limits REQUIRED
- Timeout failsafes REQUIRED

Never suggest percent output to hold load.

===============================================================================
SIMULATION
===============================================================================

Simulation must function.

- simPeriodic() supported
- No hardware-only blocking logic
- Vision fails gracefully in sim
- Commands schedulable without hardware

Simulation used to validate command flow.

===============================================================================
KNOWN ISSUES (DO NOT IGNORE)
===============================================================================

- Climber subsystem stubbed out
- VisionConstants camera mounting values (height 25.125in, angle 0) are placeholders, must be measured
- Shooter motors have NO current limits configured (safety gap, add before competition)
- Robot.java metadata string stale

Do not assume these are production-ready.

===============================================================================
WHEN UNSURE
===============================================================================

Ask:
- Practice or competition robot?
- Gravity-loaded?
- Match-critical?
- Which motor type?
- Which CAN bus?
- What does 2026 REBUILT manual specify?

Never assume.

===============================================================================
NON-FRC PROJECTS
===============================================================================

If not FRC:
- Confirm stack
- Apply secure engineering best practices

## Known Issues

- Climber subsystem is stubbed out (empty files, commented out in RobotContainer)
- `VisionConstants` camera mounting values (height 25.125in, angle 0) are placeholders, must be measured on robot
- Shooter motors have NO current limits configured. Add stator and supply limits before competition.
- `Logger.recordMetadata("ProjectName", "Elevator Simulation")` in Robot.java is stale
