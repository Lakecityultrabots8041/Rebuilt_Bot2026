package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterSubsystem extends SubsystemBase {
    
    // Speed presets (rotations per second)
    private static final double MAX_VELOCITY = 100.0;  // TODO: Tune this value
    private static final double REV_VELOCITY = 12.5;   // 1/8 of maxVel
    private static final double EJECT_VELOCITY = -5.0; // Reverse for ejecting stuck balls
    private static final double IDLE_VELOCITY = 0.0;
    
    private final TalonFX shootMotor;
    private final VelocityTorqueCurrentFOC velocityRequest;
    
    private ShooterState currentState = ShooterState.IDLE;
    
    public enum ShooterState {
        IDLE,
        REVVING,
        READY,
        EJECTING
    }

    public boolean isReady(){
        return currentState == ShooterState.READY;
    }
    
    public ShooterSubsystem() {
        shootMotor = new TalonFX(Constants.ShooterCostants.SHOOTER_MOTOR);
        velocityRequest = new VelocityTorqueCurrentFOC(0);
    }
    
   @Override
public void periodic() {
    // Apply motor control based on current state
    switch (currentState) {
        case IDLE:
            setVelocity(IDLE_VELOCITY);
            break;
        case REVVING:
            setVelocity(REV_VELOCITY);
            System.out.println("REVVING at " + REV_VELOCITY + " RPS");  // Debug
            break;
        case READY:
            setVelocity(MAX_VELOCITY);
            System.out.println("READY at " + MAX_VELOCITY + " RPS");  // Debug
            break;
        case EJECTING:
            setVelocity(EJECT_VELOCITY);
            break;
    }
}
    
    // ===== FACTORY METHODS USING LAMBDAS =====
    
    /**
     * Command factory: Spin up shooter to rev speed, then transition to ready
     */
    public Command revUp() {
    return runOnce(() -> currentState = ShooterState.REVVING)
            .andThen(Commands.waitSeconds(1.0))
            .andThen(runOnce(() -> currentState = ShooterState.READY))  // ← Wrap in runOnce!
            .withName("RevUpShooter");
}
    
    /**
     * Command factory: Set shooter to ready state at full speed
     */
    public Command shoot() {
        return runOnce(() -> currentState = ShooterState.READY)
                .withName("ShootReady");
    }
    
    /**
     * Command factory: Stop shooter motors
     */
    public Command idle() {
        return runOnce(() -> currentState = ShooterState.IDLE)
                .withName("ShooterIdle");
    }
    
    /**
     * Command factory: Reverse motors to eject stuck balls
     */
    public Command eject() {
        return runOnce(() -> currentState = ShooterState.EJECTING)
                .withName("ShooterEject");
    }
    
    /**
     * Command factory: Run shooter at specific velocity
     * @param velocityRPS velocity in rotations per second
     */
    public Command setVelocityCommand(double velocityRPS) {
        return run(() -> setVelocity(velocityRPS))
                .withName("ShooterSetVelocity_" + velocityRPS);
    }
    
    /**
     * Command factory: Shoot sequence - rev up, wait for speed, then shoot
     */
    /*public Command shootSequence() {
        return runOnce(() -> currentState = ShooterState.REVVING)
                .andThen(Commands.waitSeconds(1.0))
                .andThen(() -> currentState = ShooterState.READY)
                .andThen(Commands.waitSeconds(0.5))  // Hold shooting speed
                .finallyDo(() -> currentState = ShooterState.IDLE)
                .withName("ShootSequence");
    }*/
    
    // ===== HELPER METHODS =====
    
    private void setVelocity(double velocityRPS) {
        shootMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }
    
    /**
     * Get current shooter state
     */
    public ShooterState getState() {
        return currentState;
    }
    
    /**
     * Check if shooter is at target velocity (within tolerance)
     */
    public boolean atTargetVelocity() {
        double targetVelocity = switch (currentState) {
            case IDLE -> IDLE_VELOCITY;
            case REVVING -> REV_VELOCITY;
            case READY -> MAX_VELOCITY;
            case EJECTING -> EJECT_VELOCITY;
        };
        
        // Get current velocity from motor
        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < 2.0; // 2 RPS tolerance
    }
    
    /**
     * Command factory: Wait until shooter reaches target velocity
     */
    public Command waitUntilReady() {
        return run(() -> {})
                .until(this::atTargetVelocity)
                .withName("WaitForShooterReady");
    }
}