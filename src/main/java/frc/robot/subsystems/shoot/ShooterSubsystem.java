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

    public enum ShooterState {
        IDLE,
        REVVING,
        READY,
        EJECTING
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
        // Only send motor commands when state changes (not every loop)
        if (currentState != lastState) {
            switch (currentState) {
                case IDLE -> setVelocity(ShooterConstants.IDLE_VELOCITY);
                case REVVING -> setVelocity(ShooterConstants.REV_VELOCITY);
                case READY -> setVelocity(ShooterConstants.MAX_VELOCITY);
                case EJECTING -> setVelocity(ShooterConstants.EJECT_VELOCITY);
            }
            lastState = currentState;
        }

        // Dashboard telemetry
        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Shooter/Velocity RPS", currentVelocity);
        SmartDashboard.putNumber("Shooter/Velocity RPM", currentVelocity * 60);
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor Voltage", shootMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Current", shootMotor.getStatorCurrent().getValueAsDouble());
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

    public ShooterState getState() {
        return currentState;
    }

    public boolean atTargetVelocity() {
        double targetVelocity = switch (currentState) {
            case IDLE -> ShooterConstants.IDLE_VELOCITY;
            case REVVING -> ShooterConstants.REV_VELOCITY;
            case READY -> ShooterConstants.MAX_VELOCITY;
            case EJECTING -> ShooterConstants.EJECT_VELOCITY;
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