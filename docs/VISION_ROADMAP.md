# Vision and Auto-Score Roadmap

This document is for the programming team. It explains where the vision system is today,
why it falls short, and the architecture we are moving toward, so that next year's robot,
whatever the game is, can drive itself into a scoring position and score with a single
button, accurately and smoothly.

Read this top to bottom before touching `Limelight_Move`, `LimelightSubsystem`, or the
drivetrain. The goal is not "more vision code." The goal is to connect the systems we
already have in the right order.

## 1. What we have, and the one mismatch that hurts us

We run two independent vision pipelines and they do not talk to each other.

| Pipeline | File | Job | State |
| --- | --- | --- | --- |
| A, pose fusion | `LimelightSubsystem.updateVisionFusion()` | Blend both Limelight 4s plus wheels plus gyro into a trustworthy field pose | Solid |
| B, alignment | `Limelight_Move` | Drive the robot to a scoring spot | Servos to raw `tx` and distance, ignores pipeline A |

The core problem in one sentence: we built an excellent map (A) and then drove to score
with our eyes closed to it (B). `Limelight_Move` reacts to the camera angle to the tag
instead of where the robot is on the field. That single decision causes every symptom
below.

## 2. Why alignment is not smooth or accurate (root causes)

1. Pure proportional control, no motion profile, no feedforward. Output shrinks as we
   approach the target, so the robot creeps the last few inches and then stalls short when
   the correction drops below motor friction. That stall is the moment the driver takes
   over. See `Limelight_Move.java:117` (rotation) and `:123` (forward).
2. It controls to `tx`, not a field pose. Driving `tx` to zero means pointed at the tag,
   not centered in the scoring lane.
3. Lateral correction is commented out (`Limelight_Move.java:138`). We only close rotation
   and range, 2 of the 3 axes.
4. Single frame, no latency compensation. Raw `tx` each loop gives jittery output, so it is
   not smooth. The pose estimator that does filter and timestamp sits unused.
5. Brittle finish condition. It requires 25 consecutive in-tolerance loops
   (`VisionConstants.ALIGNED_LOOPS_REQUIRED`); a jittery P controller bounces in and out of
   tolerance and rides the 8 second timeout instead.

None of these are written wrong. They are the ceiling of the servo-to-tag approach.

## 3. The upgrade: navigate to a pose, do not servo to a tag

Use the fused robot pose we already produce, plus a target scoring pose computed from the
AprilTag field layout, and drive there with a profiled controller. A trapezoid profile
accelerates, cruises, and decelerates into the goal with feedforward, so it arrives
smoothly and actually settles.

Two ways to do it, both already available to us:

- `DriveToPose` (below), three `ProfiledPIDController` instances. Clear, self contained,
  good for learning the math.
- PathPlanner `AutoBuilder.pathfindToPose(...)`. Our `AutoBuilder` is already configured
  (`CommandSwerveDrivetrain.configureAutoBuilder`) and we ship
  `deploy/pathplanner/navgrid.json`, so obstacle-aware pathfinding to any pose is
  essentially one call. Use this once `DriveToPose` is understood.

Reference: WPILib `ProfiledPIDController` and `HolonomicDriveController` docs, and the
PathPlanner pathfinding guide.

## 4. DriveToPose command (skeleton)

```java
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import java.util.function.Supplier;

/**
 * Drives the robot to a field-relative target pose using profiled control.
 * Uses the drivetrain's fused pose estimate, so it is robust to a momentary tag
 * dropout because odometry carries us through the gap.
 */
public class DriveToPose extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> targetSupplier;   // resolved at start() so alliance is correct
    private Pose2d target;

    // TODO move gains and constraints into DriveConstants and tune with SysId numbers.
    private final ProfiledPIDController xController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(3.0, 2.0));
    private final ProfiledPIDController yController =
        new ProfiledPIDController(3.0, 0, 0, new TrapezoidProfile.Constraints(3.0, 2.0));
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(4.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    public DriveToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetSupplier) {
        this.drivetrain = drivetrain;
        this.targetSupplier = targetSupplier;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // "At goal" tolerances: about 2 cm and about 1.5 degrees. Tune on the real field.
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        thetaController.setTolerance(Math.toRadians(1.5));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        target = targetSupplier.get();
        Pose2d current = drivetrain.getState().Pose;

        // Seed each controller with the current position so the profile starts here.
        // NOTE getState().Speeds is robot-relative; for a perfectly smooth start you would
        // rotate it into the field frame before passing it as the reset velocity. Starting
        // from zero velocity is fine for a first cut.
        xController.reset(current.getX());
        yController.reset(current.getY());
        thetaController.reset(current.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d current = drivetrain.getState().Pose;

        double vx    = xController.calculate(current.getX(), target.getX());
        double vy    = yController.calculate(current.getY(), target.getY());
        double omega = thetaController.calculate(
                           current.getRotation().getRadians(),
                           target.getRotation().getRadians());

        // Convert our field-relative command into the robot-relative speeds the
        // drivetrain expects, then reuse the existing driveRobotRelative() path.
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current.getRotation());
        drivetrain.driveRobotRelative(robotRelative);
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new ChassisSpeeds()); // stop
    }
}
```

### Turning a tag into a scoring pose

```java
// Put in VisionConstants or a new TargetPoses helper.
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;

/**
 * Pose the robot should occupy to score on a tag: standoff meters in front of the
 * tag, facing it. Signs and offset must be verified on the real field.
 */
public static Pose2d scoringPoseForTag(AprilTagFieldLayout layout, int tagId, double standoffMeters) {
    Pose2d tag = layout.getTagPose(tagId).orElseThrow().toPose2d();
    Transform2d inFrontFacingTag =
        new Transform2d(new Translation2d(standoffMeters, 0.0), Rotation2d.k180deg);
    return tag.transformBy(inFrontFacingTag);
}
```

Why a `Supplier<Pose2d>` and not a `Pose2d`? Same reason we pass `::getHubTags`: the
target depends on alliance and on which tag we currently see, which is not known until the
command starts. Resolve it in `initialize()`, not in the constructor.

## 5. The scoring state machine (game agnostic)

The whole point: the drive-to-pose, the pose estimator, and the sequence do not change
year to year. Only two things swap: how we get ready, and how we score. We capture those
two behind interfaces.

```java
package frc.robot.scoring;

import edu.wpi.first.wpilibj2.command.Command;

/** Anything that tells us we are holding a game piece. */
public interface GamePieceSensor {
    boolean hasPiece();
}

/**
 * A scoring mechanism. This year it is the shooter. Next year it is an
 * arm, elevator, or gripper. The auto-score sequence never needs to know which.
 */
public interface Scorer {
    Command prepare();          // spin the flywheel up, or raise the arm to node height
    boolean readyToScore();     // flywheel at RPS, or arm at position
    Command release();          // feed the ball, or open the gripper
    default boolean pieceGone() { return true; } // optional, confirm the piece actually left
}
```

```java
package frc.robot.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

import java.util.function.Supplier;

public final class AutoScore {
    private AutoScore() {}

    /**
     * Drive to the scoring pose while preparing the mechanism, wait until both the
     * pose and the mechanism are ready, score, then confirm the piece left.
     * Aborts immediately if we are not actually holding a game piece.
     */
    public static Command autoScore(CommandSwerveDrivetrain drivetrain,
                                    Supplier<Pose2d> targetPose,
                                    GamePieceSensor sensor,
                                    Scorer scorer) {
        Command scoreSequence = Commands.sequence(
            // Approach and spin-up happen at the same time. This is what makes it fast.
            Commands.parallel(
                new DriveToPose(drivetrain, targetPose),
                scorer.prepare()
            ),
            // Gate: do not release until the mechanism is genuinely ready.
            Commands.waitUntil(scorer::readyToScore).withTimeout(2.0),
            scorer.release(),
            Commands.waitUntil(scorer::pieceGone).withTimeout(1.0)
        );

        return Commands.either(
            scoreSequence,
            Commands.print("AutoScore aborted: no game piece"),
            sensor::hasPiece
        ).withName("AutoScore");
    }
}
```

### This year's wiring (shooter)

```java
Scorer shooterScorer = new Scorer() {
    public Command prepare()       { return shooter.readyFlywheel(); }
    public boolean readyToScore()  { return shooter.isFlywheelReady(); }
    public Command release()       { return shooter.startFeeding(); }
    public boolean pieceGone()     { return true; } // no exit sensor yet, see section 6
};
```

### Next year's wiring (placement, illustrative)

```java
Scorer placeScorer = new Scorer() {
    public Command prepare()       { return superstructure.goToNode(); }
    public boolean readyToScore()  { return superstructure.atNode(); }
    public Command release()       { return gripper.open(); }
    public boolean pieceGone()     { return !gripperBeamBreak.hasPiece(); }
};
```

Same `autoScore(...)`. That is the payoff of the architecture.

## 6. Sensors: "do I have a piece?" and "am I in range?"

These are two different questions and need different answers.

### Possession, "do I have a piece?"

| Sensor | How | Bus and cost | Use it for |
| --- | --- | --- | --- |
| Beam-break (IR) | Piece blocks beam, digital signal | roboRIO DIO, about 10 to 25 USD | The default. Reliable binary "piece present". |
| CAN time-of-flight (CTRE CANrange, PWF TOF) | Distance | CAN | Where the piece is, not just present. Index to an exact spot. |
| Motor stator current | Current spikes on grab | Free, TalonFX already reports it | Backup confirmation. Noisy alone, good in combination. |
| Color and proximity (REV V3) | Color plus proximity | I2C | When pieces have a color or we must reject the wrong alliance's. |
| Limit switch or lever | Contact when seated | DIO, about 5 USD | Dead simple "fully seated". |

Starter implementation:

```java
public class BeamBreak implements GamePieceSensor {
    private final edu.wpi.first.wpilibj.DigitalInput input;
    public BeamBreak(int dioPort) { input = new edu.wpi.first.wpilibj.DigitalInput(dioPort); }
    // Wiring dependent: many beam-breaks read LOW when broken. Verify on the bench.
    @Override public boolean hasPiece() { return !input.get(); }
}
```

```java
// Costs nothing extra. We already read stator current on the flywheel and intake.
public class CurrentSpikeSensor implements GamePieceSensor {
    private final com.ctre.phoenix6.hardware.TalonFX motor;
    private final double thresholdAmps;
    public CurrentSpikeSensor(com.ctre.phoenix6.hardware.TalonFX m, double amps) {
        this.motor = m; this.thresholdAmps = amps;
    }
    @Override public boolean hasPiece() {
        return motor.getStatorCurrent().getValueAsDouble() > thresholdAmps;
    }
}
```

### In range or in position, "can I score from here?"

- Shooting game: distance to goal from the AprilTag pose is within the shooter's effective
  table. We already compute this (`ShooterConstants.getSpeedForDistance`).
- Placement game: "in range" does not really exist. It is "am I at the pose", which comes
  from `DriveToPose`, plus a forward TOF or CANrange for the final 2 cm of standoff.
- Finding a piece on the floor (auto-intake): that is Limelight neural-net object
  detection, not AprilTags. A different pipeline to add later.

## 7. Engineering-philosophy budget (what the architecture actually needs)

The expensive part of this roadmap is time, not parts. Everything in sections 3 to 5 runs
on hardware we already own: two Limelight 4s, the swerve base, TalonFX current reporting.

Minimum to make auto-score real and reliable:

- One beam-break per game piece path (about 10 to 25 USD). This single cheap sensor
  unlocks the whole `GamePieceSensor` gate and removes most "did it actually have a ball?"
  failures.
- Optional, one CAN time-of-flight (about 40 to 70 USD), only if the game needs precise
  indexing or close-range placement standoff.

Everything else is free: stator-current sensing, PathPlanner pathfinding, pose fusion.

The FIRST lesson worth teaching the team: we are not buying our way to a better robot. The
biggest gain this offseason, connecting the pose estimator to a profiled drive-to-pose,
costs zero dollars. Spend on the one sensor that removes a failure mode, not on hardware
that papers over a software gap.

## 8. Skills to learn (offseason study list)

1. `ProfiledPIDController` and `TrapezoidProfile`, why profiled control settles and pure P
   does not.
2. `AutoBuilder.pathfindToPose()`, turn our existing PathPlanner setup into one-call
   drive-to-score.
3. `Pose2d` and `Transform2d` math, tag pose to scoring pose.
4. Wiring a DIO beam-break and reading the stator current we already have.
5. Latency compensation with measurement timestamps. We already do it in pose fusion, so
   extend the habit to control.

## 9. Suggested order of work

1. Add a `GamePieceSensor` (beam-break) and prove `hasPiece()` on the bench.
2. Write `DriveToPose`; tune it driving to a fixed field pose in sim, then on the field.
3. Add `scoringPoseForTag(...)`; drive to a real tag's scoring pose.
4. Wrap the shooter in a `Scorer`; wire `AutoScore` to one button.
5. Only then swap `Limelight_Move` out of the main alignment binding.
6. Once trusted, replace the `DriveToPose` internals with `pathfindToPose` for obstacle
   avoidance.
