package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommands {
    
    // Private constructor to prevent instantiation
    private ShooterCommands() {
        throw new UnsupportedOperationException("This is a utility class");
    }
    
    // ===== BASIC COMMANDS =====
    
    /**
     * Rev up the shooter to prepare for shooting
     */
    public static Command revUp(ShooterSubsystem shooter) {
        return shooter.revUp();
    }
    
    /**
     * Set shooter to ready/shooting state
     */
    public static Command shoot(ShooterSubsystem shooter) {
        return shooter.shoot();
    }
    
    /**
     * Stop the shooter motors
     */
    public static Command idle(ShooterSubsystem shooter) {
        return shooter.idle();
    }
    
    /**
     * Reverse motors to eject stuck balls
     */
    public static Command eject(ShooterSubsystem shooter) {
        return shooter.eject();
    }
    
    // ===== COMPOSITE COMMANDS =====
    
    /**
     * Complete shooting sequence: rev up, wait for ready, shoot, then idle
     */
    public static Command shootSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("Starting shoot sequence"), shooter),
            shooter.revUp(),
            shooter.waitUntilReady(),
            Commands.waitSeconds(0.5),  // Hold shooting speed
            shooter.idle()
        ).withName("CompleteShootSequence");
    }
    
    /**
     * Quick shoot - assumes shooter is already revved
     */
    public static Command quickShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.shoot(),
            Commands.waitSeconds(0.5),
            shooter.idle()
        ).withName("QuickShoot");
    }
    
    /**
     * Emergency eject sequence - eject for specified duration then idle
     */
    public static Command ejectSequence(ShooterSubsystem shooter, double durationSeconds) {
        return Commands.sequence(
            shooter.eject(),
            Commands.waitSeconds(durationSeconds),
            shooter.idle()
        ).withName("EjectSequence");
    }
    
    /**
     * Smart shoot - only revs if not already ready
     */
    public static Command smartShoot(ShooterSubsystem shooter) {
        return Commands.either(
            // If already ready, just shoot
            quickShoot(shooter),
            // Otherwise, do full sequence
            shootSequence(shooter),
            // Condition: is shooter already at target velocity?
            shooter::atTargetVelocity
        ).withName("SmartShoot");
    }
    
    /**
     * Rev and hold - keeps shooter at ready state indefinitely
     */
    public static Command revAndHold(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.revUp(),
            shooter.waitUntilReady()
        ).andThen(Commands.run(() -> {}, shooter))  // Hold state
        .withName("RevAndHold");
    }
    
    /**
     * Toggle shooter between idle and ready
     */
    public static Command toggle(ShooterSubsystem shooter) {
        return Commands.either(
            shooter.idle(),
            shooter.revUp(),
            () -> shooter.getState() != ShooterSubsystem.ShooterState.IDLE
        ).withName("ToggleShooter");
    }
}