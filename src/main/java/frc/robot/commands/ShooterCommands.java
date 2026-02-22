package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.ShooterSubsystem;

public class ShooterCommands extends Command {

    private ShooterCommands() {}

    // ===== BASIC COMMANDS =====

    /** Spin flywheel to pre-rev speed. */
    public static Command revUp(ShooterSubsystem shooter) {
        return shooter.revFlywheel();
    }

    /**
     * Flywheel to full speed + feed rollers on.
     * Used for the teleop trigger — starts everything at once.
     * For autos, use shootSequence() which waits for flywheel speed first.
     */
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
     * Full shoot sequence for autonomous:
     *   1. Spin flywheel to full speed
     *   2. Wait until flywheel actually reaches that speed
     *   3. Turn feed rollers on — ball enters flywheel and launches
     *   4. Wait for ball to clear
     *   5. Stop everything
     *
     * The flywheel-first order is what makes shots consistent.
     * If the ball enters before the flywheel is at speed, the shot will be short.
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

    /**
     * Quick shoot — skips the rev wait.
     * Only use this when the flywheel is already at speed.
     */
    public static Command quickShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.startFeeding(),
            Commands.waitSeconds(0.5),
            shooter.idleAll()
        ).withName("QuickShoot");
    }

    /**
     * Pass sequence — flywheel and feed to passing speed, wait, idle.
     * Passing is lower precision than shooting so both start together.
     */
    public static Command passSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.passAll(),
            Commands.waitSeconds(1.0),
            shooter.idleAll()
        ).withTimeout(3.0)
         .withName("PassSequence");
    }

    /**
     * Eject sequence — reverses feed rollers to clear a stuck ball.
     * Flywheel stays off.
     */
    public static Command ejectSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.ejectFeed(),
            Commands.waitSeconds(0.75),
            shooter.idleAll()
        ).withName("EjectSequence");
    }

    /**
     * Smart shoot — if flywheel is already at speed, just feed.
     * Otherwise does the full rev + wait + feed sequence.
     */
    public static Command smartShoot(ShooterSubsystem shooter) {
        return Commands.either(
            quickShoot(shooter),
            shootSequence(shooter),
            shooter::isFlywheelReady
        ).withName("SmartShoot");
    }
}
