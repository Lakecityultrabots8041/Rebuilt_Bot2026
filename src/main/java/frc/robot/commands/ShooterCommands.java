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
        return Commands.sequence(shooter.shoot());
    }

    //TODO Review code here as well
    public static Command speedSwitch(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.speedSwitch());
    }

    /*
     * QUESTION: Why does testShot exist when we already have shootSequence()
     * and smartShoot()? This calls testDelayedShot() which blindly waits 1.5s
     * instead of checking real flywheel velocity.
     *
     * If you wanted a "delayed shot" that waits for the flywheel, you already
     * have better options in this file:
     *
     *   shootSequence()  - revs flywheel, waits until ACTUALLY at speed, then
     *                      feeds. Best for autos or anytime you need a guaranteed
     *                      full-speed shot.
     *
     *   smartShoot()     - checks if flywheel is already spinning. If yes, feeds
     *                      immediately. If no, does the full shootSequence().
     *                      Best "just do the right thing" option.
     *
     *   quickShoot()     - skips the wait entirely. Only safe when auto-aim has
     *                      already pre-spun the flywheel to speed.
     *
     * Pick one of these instead. All three check real velocity or assume it is
     * already handled, rather than guessing with a hardcoded delay.
     */
    // public static Command testShot(ShooterSubsystem shooter) {
    //     return Commands.sequence(shooter.testDelayedShot());
    // }

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
