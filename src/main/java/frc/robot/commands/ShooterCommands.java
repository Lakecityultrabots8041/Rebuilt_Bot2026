package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shoot.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.commands.Limelight_Move;


public class ShooterCommands extends Command {

    private ShooterCommands() {

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
        Commands.runOnce(() -> System.out.println("Starting shoot sequence")),
        shooter.revUp(),                    // Actually rev up!
        shooter.waitUntilReady(),          // Wait for speed
        Commands.waitSeconds(0.5),         // Hold shooting speed
        shooter.idle(), 
        Commands.runOnce(() -> System.out.println("Commplete shoot sequence"))                    // Stop
    ).withTimeout(3.0)
     .withName("CompleteShootSequence");
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
    public static Command ejectSequence(ShooterSubsystem shooter) {
        return Commands.sequence(
            Commands.runOnce(() -> System.out.println("Starting Eject Sequence")),
            shooter.eject(),
            Commands.waitSeconds(0.5),
            shooter.idle(),
            Commands.runOnce(() -> System.out.println("Ending Eject Sequence"))
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
     
    public static Command revAndHold(ShooterSubsystem shooter) {
        return Commands.sequence(
            shooter.revUp(),
            shooter.waitUntilReady()
        ).andThen(Commands.run(() -> {}, shooter))  // Hold state
        .withName("RevAndHold");
    }*/
    
    /**
     * Toggle shooter between idle and ready
     */
   /* public static Command toggle(ShooterSubsystem shooter) {
        return Commands.either(
            shooter.idle(),
            shooter.revUp(),
            () -> shooter.getState() != ShooterSubsystem.ShooterState.IDLE
        ).withName("ToggleShooter");
    }*/

   
   
   /*  private Command alignAndShoot() {
        return Commands.sequence(
            alignToTag.until(() -> limelight.isAligned() && limelight.atTargetDistance)
                .withTimeout(3.0)

        new ParallelCommandGroup(
                alignToTag,
                shooterSubsystem.revUpAndWait()
            ).withTimeout(3.0),

            Commands.parallel(
                alignToTag,
                ShooterCommands.quickShoot(shooterSubsystem)
            )
        ).withName("Align and Shoot");
    }*/

   
}