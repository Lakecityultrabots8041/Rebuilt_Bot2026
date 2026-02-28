package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.ShooterSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {}

    // ===== BASIC COMMANDS =====

    /** Spin flywheel to pre-rev speed. */
    public static Command revUpFlywheel(ShooterSubsystem shooter) {
       return Commands.sequence(shooter.revFlywheel(), shooter.waitUntilFlywheelReady());
    }

    /** Teleop trigger. Flywheel + feed start at the same time. For autos use shootSequence(). */
    public static Command shoot(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.shoot(), shooter.startLo4d());
    }

    /** Stop everything. */
    public static Command idle(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.idleAll(), shooter.stopLo4d());
    }

    /** Run feed rollers in reverse to clear a stuck ball. */
    public static Command eject(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.ejectFeed(), shooter.ejectLo4d());
    }

    /** Flywheel and feed to passing speed/power. */
    public static Command pass(ShooterSubsystem shooter) {
        return shooter.passAll();
    }

    // ===== SEQUENCES =====

    // Auto shoot: rev -> wait for speed -> feed -> wait for ball to clear -> stop.
    // Flywheel has to be at speed first or the shot will be short.
    public static Command shootSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.readyFlywheel(),
            shooter.waitUntilFlywheelReady(),
            shooter.startFeeding(),
            shooter.startLo4d(),
            Commands.waitSeconds(0.5),
            shooter.idleAll()
        ).withTimeout(4.0)
         .withName("ShootSequence");
    }

    /** Skips the flywheel wait. Only use when flywheel is already at speed. */
    public static Command quickShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.readyFlywheel(),
            shooter.startFeeding(),
            shooter.startLo4d(),
            Commands.waitSeconds(0.5),
            shooter.idleAll()
        ).withName("QuickShoot");
    }

    /** Pass: flywheel and feed start together, wait, then stop. */
    public static Command passSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.passAll(),
            Commands.waitSeconds(1.0),
            shooter.idleAll()
        ).withTimeout(3.0)
         .withName("PassSequence");
    }

    /** Reverse feed rollers to clear a stuck ball. Flywheel stays off. */
    public static Command ejectSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.ejectFeed(),
            shooter.ejectLo4d(),
            Commands.waitSeconds(0.75),
            shooter.idleAll()
        ).withName("EjectSequence");
    }

    /** If flywheel is at speed, just feed. Otherwise does the full shoot sequence. */
    public static Command smartShoot(ShooterSubsystem shooter) {
        return Commands.either(
            quickShoot(shooter),
            shootSequence(shooter),
            shooter::isFlywheelReady
        ).withName("SmartShoot");
    }
}
