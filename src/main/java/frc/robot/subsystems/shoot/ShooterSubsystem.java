package frc.robot.subsystems.shoot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX shootMotor;
    private final VelocityTorqueCurrentFOC velocityRequest;

    private ShooterState currentState = ShooterState.IDLE;
    private ShooterState lastState = null;
    private double targetVariableVelocity = 0;

    public enum ShooterState {
        IDLE,
        REVVING,
        READY,
        EJECTING,
        VARIABLE
    }

    public boolean isReady() {
        return currentState == ShooterState.READY;
    }

    public ShooterSubsystem() {
        shootMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR, ShooterConstants.CANIVORE_NAME);
        velocityRequest = new VelocityTorqueCurrentFOC(0);

        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = ShooterConstants.kP;
        configs.Slot0.kV = ShooterConstants.kV;
        configs.Slot0.kS = ShooterConstants.kS;

        shootMotor.getConfigurator().apply(configs);
    }

    @Override
    public void periodic() {
        // VARIABLE state sends velocity every loop (target changes continuously)
        if (currentState == ShooterState.VARIABLE) {
            setVelocity(targetVariableVelocity);
        } else if (currentState != lastState) {
            // Other states only send motor commands when state changes
            switch (currentState) {
                case IDLE -> setVelocity(ShooterConstants.IDLE_VELOCITY);
                case REVVING -> setVelocity(ShooterConstants.REV_VELOCITY);
                case READY -> setVelocity(ShooterConstants.MAX_VELOCITY);
                case EJECTING -> setVelocity(ShooterConstants.EJECT_VELOCITY);
                default -> {}
            }
        }
        lastState = currentState;

        // Dashboard telemetry
        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Shooter/Velocity RPS", currentVelocity);
        SmartDashboard.putNumber("Shooter/Velocity RPM", currentVelocity * 60);
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor Voltage", shootMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Current", shootMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Target Variable RPS", targetVariableVelocity);
    }

    // ===== COMMAND FACTORIES =====

    public Command revUp() {
        return runOnce(() -> currentState = ShooterState.REVVING)
                .andThen(Commands.waitSeconds(1.0))
                .andThen(runOnce(() -> currentState = ShooterState.READY))
                .withName("RevUpShooter");
    }

    public Command shoot() {
        return runOnce(() -> currentState = ShooterState.READY)
                .withName("ShootReady");
    }

    public Command idle() {
        return runOnce(() -> currentState = ShooterState.IDLE)
                .withName("ShooterIdle");
    }

    public Command eject() {
        return runOnce(() -> currentState = ShooterState.EJECTING)
                .withName("ShooterEject");
    }

    public Command setVelocityCommand(double velocityRPS) {
        return run(() -> setVelocity(velocityRPS))
                .withName("ShooterSetVelocity_" + velocityRPS);
    }

    // ===== HELPER METHODS =====

    private void setVelocity(double velocityRPS) {
        shootMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    /**
     * Sets the shooter to VARIABLE state with a continuously-updating target velocity.
     * Called directly (not a command) to avoid scheduling conflicts with auto-aim.
     */
    public void setVariableVelocity(double rps) {
        targetVariableVelocity = rps;
        currentState = ShooterState.VARIABLE;
    }

    public ShooterState getState() {
        return currentState;
    }

    public boolean atTargetVelocity() {
        double targetVelocity = switch (currentState) {
            case IDLE -> ShooterConstants.IDLE_VELOCITY;
            case REVVING -> ShooterConstants.REV_VELOCITY;
            case READY -> ShooterConstants.MAX_VELOCITY;
            case EJECTING -> ShooterConstants.EJECT_VELOCITY;
            case VARIABLE -> targetVariableVelocity;
        };

        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(this::atTargetVelocity)
                .withTimeout(ShooterConstants.READY_TIMEOUT_SECONDS)
                .withName("WaitForShooterReady");
    }
}