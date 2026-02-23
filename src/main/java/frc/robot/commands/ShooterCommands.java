package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.ShooterSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {}

    // ===== BASIC COMMANDS =====

    /** Spin flywheel to pre-rev speed. */
    public static Command revUp(ShooterSubsystem shooter) {
        return shooter.revFlywheel();
    }

    /** Teleop trigger. Flywheel + feed start at the same time. For autos use shootSequence(). */
    public static Command shoot(ShooterSubsystem shooter) {
        return shooter.shoot();
    }

    /** Stop everything. */
    public static Command idle(ShooterSubsystem shooter) {
        return shooter.idleAll();
    }

    /** Run feed rollers in reverse to clear a stuck ball. */
    public static Command eject(ShooterSubsystem shooter) {
        return shooter.ejectFeed();
    }

    /** Flywheel and feed to passing speed/power. */
    public static Command pass(ShooterSubsystem shooter) {
        return shooter.passAll();
    }

    // ===== SEQUENCES =====

    /**
     * Auto shoot sequence:
     *   1. Spin flywheel to full speed
     *   2. Wait for it to get there
     *   3. Feed rollers push ball into flywheel
     *   4. Wait for ball to clear
     *   5. Stop everything
     *
     * Flywheel has to be at speed first or the shot will be short.
     */
    public static Command shootSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.readyFlywheel(),
            shooter.waitUntilFlywheelReady(),
            shooter.startFeeding(),
            Commands.waitSeconds(0.5),
            shooter.idleAll()
        ).withTimeout(4.0)
         .withName("ShootSequence");
    }

    /** Skips the flywheel wait. Only use when flywheel is already at speed. */
    public static Command quickShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.startFeeding(),
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
