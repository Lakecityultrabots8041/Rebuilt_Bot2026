package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.ShooterSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {}

    // ===== BASIC COMMANDS =====

    public static Command revUpFlywheel(ShooterSubsystem shooter) {
       return Commands.sequence(shooter.revFlywheel(), shooter.waitUntilFlywheelReady());
    }

    public static Command shoot(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.shoot());
    }

    public static Command idle(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.idleAll(), shooter.stopLo4d());
    }

    public static Command eject(ShooterSubsystem shooter) {
        return Commands.sequence(shooter.ejectFeed(), shooter.ejectLo4d());
    }

    public static Command pass(ShooterSubsystem shooter) {
        return shooter.passAll();
    }

    // ===== SEQUENCES =====
    // See SHOOTER_TUNING.md for when to use each one.

    // Waits for flywheel to reach speed before feeding. Best for autos.
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

    // Skips flywheel wait. Only safe when flywheel is already at speed.
    public static Command quickShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.readyFlywheel(),
            shooter.startFeeding(),
            shooter.startLo4d(),
            Commands.waitSeconds(4.5),
            shooter.idleAll()
        ).withName("QuickShoot");
    }

    public static Command passSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.passAll(),
            Commands.waitSeconds(1.0),
            shooter.idleAll()
        ).withTimeout(3.0)
         .withName("PassSequence");
    }

    public static Command ejectSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.ejectFeed(),
            shooter.ejectLo4d(),
            Commands.waitSeconds(0.75),
            shooter.idleAll()
        ).withName("EjectSequence");
    }

    // Checks if flywheel is already spinning. If yes, just feeds. If no, full sequence.
    public static Command smartShoot(ShooterSubsystem shooter) {
        return Commands.either(
            quickShoot(shooter),
            shootSequence(shooter),
            shooter::isFlywheelReady
        ).withName("SmartShoot");
    }
}
