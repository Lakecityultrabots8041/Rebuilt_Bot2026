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
    private final TalonFX flywheelMotor;
    private final VelocityTorqueCurrentFOC velocityRequest;

    private ShooterState currentState = ShooterState.IDLE;
    private ShooterState flywheelState = ShooterState.FLYWHELLIDLE; 
    private ShooterState lastState = null; 
    private ShooterState lastFlywheelState = null;

    public enum ShooterState {
        IDLE,
        FLYWHELLIDLE,
        REVVING,
        FLYWHELLREVVING,
        READY,
        FLYWHEELREADY,
        FLYWHEELEJECTING,
        EJECTING
    }

    public boolean isReady() {
        return currentState == ShooterState.READY && flywheelState == ShooterState.FLYWHEELREADY;
    }

    public ShooterSubsystem() {
        shootMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR, ShooterConstants.CANIVORE_NAME);
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR, ShooterConstants.CANIVORE_NAME);
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

        // Flywheel states
        if (flywheelState != lastFlywheelState) {
            switch (flywheelState) {
                case FLYWHELLIDLE -> setVelocity(ShooterConstants.FLYWHEEL_IDLE_VELOCITY);
                case FLYWHELLREVVING -> setVelocity(ShooterConstants.FLYWHEEL_REV_VELOCITY);
                case FLYWHEELREADY -> setVelocity(ShooterConstants.FLYWHEEL_MAX_VELOCITY);
            }
            lastFlywheelState = flywheelState;
        }

        // Dashboard telemetry
        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        SmartDashboard.putNumber("Shooter/Velocity RPS", currentVelocity);
        SmartDashboard.putNumber("Shooter/Velocity RPM", currentVelocity * 60);
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putString("Shooter/Flywheel State", flywheelState.toString());
        
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity() && atFlywheelTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor Voltage", shootMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Current", shootMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Slywheeler/Motor Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Flywheeler/Motor Current", flywheelMotor.getStatorCurrent().getValueAsDouble());
    }

    // ===== COMMAND FACTORIES =====

    public Command revUp() {
        return runOnce(() -> {
            currentState = ShooterState.REVVING;
            flywheelState = ShooterState.FLYWHELLREVVING;
        })
                .andThen(Commands.waitSeconds(1.0))
                .andThen(runOnce(() -> {
                    currentState = ShooterState.READY;
                    flywheelState = ShooterState.FLYWHEELREADY;
                }))
                .withName("RevUpShooter");
    }

    public Command shoot() {
        return runOnce(() -> {
            currentState = ShooterState.READY;
            flywheelState = ShooterState.FLYWHEELREADY;
        })
                .withName("ShootReady");
    }

    public Command idle() {
        return runOnce(() -> {
            currentState = ShooterState.IDLE;
            flywheelState = ShooterState.FLYWHELLIDLE;
        })
                .withName("ShooterIdle");
    }

    public Command eject() {
        return runOnce(() -> {
            currentState = ShooterState.EJECTING;
            flywheelState = ShooterState.FLYWHEELEJECTING;
        })
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

    public ShooterState getFlywheelState() {
        return flywheelState;
    }
    public boolean atTargetVelocity() {
        double targetVelocity = switch (currentState) {
            case IDLE -> ShooterConstants.IDLE_VELOCITY;
            case REVVING -> ShooterConstants.REV_VELOCITY;
            case READY -> ShooterConstants.MAX_VELOCITY;
            case EJECTING -> ShooterConstants.EJECT_VELOCITY;
            default -> 0.0;
        };

        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    }

    public boolean atFlywheelTargetVelocity() {
        double targetVelocity = switch (flywheelState) {
            case FLYWHELLIDLE -> ShooterConstants.FLYWHEEL_IDLE_VELOCITY;
            case FLYWHELLREVVING -> ShooterConstants.FLYWHEEL_REV_VELOCITY;
            case FLYWHEELREADY -> ShooterConstants.FLYWHEEL_MAX_VELOCITY;
            case FLYWHEELEJECTING -> ShooterConstants.FLYWHEEL_IDLE_VELOCITY;
            default -> 0.0;
        };

        double currentVelocity = flywheelMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(() -> atTargetVelocity() && atFlywheelTargetVelocity())
                .withTimeout(ShooterConstants.READY_TIMEOUT_SECONDS)
                .withName("WaitForShooterAndFlywheelReady");
    }
}   