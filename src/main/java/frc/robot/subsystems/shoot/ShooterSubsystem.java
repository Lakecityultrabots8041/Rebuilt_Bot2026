package frc.robot.subsystems.shoot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;


public class ShooterSubsystem extends SubsystemBase {


    private final TalonFX shootMotor;
    private final TalonFX flywheelMotor;
    private final VelocityTorqueCurrentFOC velocityRequest;

    // Pre-registered status signals — bulk-refreshed once per loop
    private final StatusSignal<AngularVelocity> shooterVelocitySig;
    private final StatusSignal<AngularVelocity> flywheelVelocitySig;
    private final StatusSignal<Voltage> shooterVoltageSig;
    private final StatusSignal<Current> shooterCurrentSig;
    private final StatusSignal<Voltage> flywheelVoltageSig;
    private final StatusSignal<Current> flywheelCurrentSig;

    // Track what we're commanding so we can display it in sim
    private double lastCommandedShooterRPS = 0;
    private double lastCommandedFlywheelRPS = 0;

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
        EJECTING,
        PASSING,
        FLYWHEELPASS,
        VISION_TRACKING
    }

    public boolean isReady() {
        return currentState == ShooterState.READY && flywheelState == ShooterState.FLYWHEELREADY;
    }

    public ShooterSubsystem() {
        shootMotor = new TalonFX(ShooterConstants.SHOOTER_MOTOR, ShooterConstants.CANIVORE);
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR, ShooterConstants.CANIVORE);
        velocityRequest = new VelocityTorqueCurrentFOC(0);

        var shooterConfigs = new TalonFXConfiguration();
        shooterConfigs.Slot0.kP = ShooterConstants.kP;
        shooterConfigs.Slot0.kV = ShooterConstants.kV;
        shooterConfigs.Slot0.kS = ShooterConstants.kS;
        shootMotor.getConfigurator().apply(shooterConfigs);

        var flywheelConfigs = new TalonFXConfiguration();
        flywheelConfigs.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        flywheelConfigs.Slot0.kV = ShooterConstants.FLYWHEEL_kV;
        flywheelConfigs.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        flywheelMotor.getConfigurator().apply(flywheelConfigs);

        // Register status signals once — bulk-refreshed in periodic()
        shooterVelocitySig = shootMotor.getVelocity();
        flywheelVelocitySig = flywheelMotor.getVelocity();
        shooterVoltageSig = shootMotor.getMotorVoltage();
        shooterCurrentSig = shootMotor.getStatorCurrent();
        flywheelVoltageSig = flywheelMotor.getMotorVoltage();
        flywheelCurrentSig = flywheelMotor.getStatorCurrent();
    }

    @Override
    public void periodic() {
        // Bulk-refresh all signals once — single CAN round-trip
        BaseStatusSignal.refreshAll(
            shooterVelocitySig, flywheelVelocitySig,
            shooterVoltageSig, shooterCurrentSig,
            flywheelVoltageSig, flywheelCurrentSig);

        // VARIABLE state: update both motors every loop (speed changes with distance)
        if (currentState == ShooterState.VISION_TRACKING) {
            setVelocity(variableVelocityRPS);
            setFlywheelVelocity(variableVelocityRPS * ShooterConstants.FLYWHEEL_SPEED_RATIO);
        } else {
            // Only send motor commands when state changes (not every loop)
            if (currentState != lastState) {
                switch (currentState) {
                    case IDLE    -> setVelocity(ShooterConstants.IDLE_VELOCITY);
                    case REVVING -> setVelocity(ShooterConstants.REV_VELOCITY);
                    case READY   -> setVelocity(ShooterConstants.MAX_VELOCITY);
                    case EJECTING -> setVelocity(ShooterConstants.EJECT_VELOCITY);
                    case PASSING -> setVelocity(ShooterConstants.PASS_VELOCITY);
                    default -> {}
                }
                lastState = currentState;
            }

            // Flywheel states
            if (flywheelState != lastFlywheelState) {
                switch (flywheelState) {
                    case FLYWHELLIDLE    -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_IDLE_VELOCITY);
                    case FLYWHELLREVVING -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_REV_VELOCITY);
                    case FLYWHEELREADY   -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_MAX_VELOCITY);
                    case FLYWHEELPASS    -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_PASS_VELOCITY);
                    default -> {}
                }
                lastFlywheelState = flywheelState;
            }
        }

        // Read cached signal values (already refreshed above)
        double actualShooterVelocity = shooterVelocitySig.getValueAsDouble();
        double actualFlywheelVelocity = flywheelVelocitySig.getValueAsDouble();

        // Dashboard telemetry
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putString("Shooter/Flywheel State", flywheelState.toString());
        SmartDashboard.putNumber("Shooter/Commanded RPS", lastCommandedShooterRPS);
        SmartDashboard.putNumber("Shooter/Commanded RPM", lastCommandedShooterRPS * 60);
        SmartDashboard.putNumber("Flywheel/Commanded RPS", lastCommandedFlywheelRPS);
        SmartDashboard.putNumber("Flywheel/Commanded RPM", lastCommandedFlywheelRPS * 60);

        if (currentState == ShooterState.VISION_TRACKING) {
            SmartDashboard.putNumber("Shooter/Vision Target RPS", variableVelocityRPS);
        }

        SmartDashboard.putNumber("Shooter/Actual RPS", actualShooterVelocity);
        SmartDashboard.putNumber("Flywheel/Actual RPS", actualFlywheelVelocity);
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity() && atFlywheelTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor Voltage", shooterVoltageSig.getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Motor Current", shooterCurrentSig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Motor Voltage", flywheelVoltageSig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Motor Current", flywheelCurrentSig.getValueAsDouble());
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

    public Command pass() {
        return runOnce(() -> {
            currentState = ShooterState.PASSING;
            flywheelState = ShooterState.FLYWHEELPASS;
        })
                .withName("ShooterPass");
    }

    public Command setVelocityCommand(double velocityRPS) {
        return run(() -> setVelocity(velocityRPS))
                .withName("ShooterSetVelocity_" + velocityRPS);
    }

    // ===== VARIABLE VELOCITY (Limelight auto-aim) =====

    private double variableVelocityRPS = 0;

    /**
     * Called each loop by auto-aim to set shooter speed based on Limelight distance.
     * Bypasses the state machine and directly controls both motors.
     */
    public void setVariableVelocity(double velocityRPS) {
        variableVelocityRPS = velocityRPS;
        if (currentState != ShooterState.VISION_TRACKING) {
            currentState = ShooterState.VISION_TRACKING;
            flywheelState = ShooterState.VISION_TRACKING;
        }
    }

    /**
     * Stops variable velocity mode and returns to IDLE.
     * Called when auto-aim disengages.
     */
    public void clearVariableVelocity() {
        if (currentState == ShooterState.VISION_TRACKING) {
            variableVelocityRPS = 0;
            currentState = ShooterState.IDLE;
            flywheelState = ShooterState.FLYWHELLIDLE;
            lastState = null;
            lastFlywheelState = null;
        }
    }

    // ===== HELPER METHODS =====

    private void setVelocity(double velocityRPS) {
        lastCommandedShooterRPS = velocityRPS;
        shootMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    private void setFlywheelVelocity(double velocityRPS) {
        lastCommandedFlywheelRPS = velocityRPS;
        flywheelMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    public ShooterState getState() {
        return currentState;
    }

    public ShooterState getFlywheelState() {
        return flywheelState;
    }
    public boolean atTargetVelocity() {
        double targetVelocity = switch (currentState) {
            case IDLE    -> ShooterConstants.IDLE_VELOCITY;
            case REVVING -> ShooterConstants.REV_VELOCITY;
            case READY   -> ShooterConstants.MAX_VELOCITY;
            case EJECTING -> ShooterConstants.EJECT_VELOCITY;
            case PASSING  -> ShooterConstants.PASS_VELOCITY;
            case VISION_TRACKING -> variableVelocityRPS;
            default -> 0.0;
        };

        double currentVelocity = shooterVelocitySig.getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    }

    public boolean atFlywheelTargetVelocity() {
        double targetVelocity = switch (flywheelState) {
            case FLYWHELLIDLE    -> ShooterConstants.FLYWHEEL_IDLE_VELOCITY;
            case FLYWHELLREVVING -> ShooterConstants.FLYWHEEL_REV_VELOCITY;
            case FLYWHEELREADY   -> ShooterConstants.FLYWHEEL_MAX_VELOCITY;
            case FLYWHEELEJECTING -> ShooterConstants.FLYWHEEL_IDLE_VELOCITY;
            case FLYWHEELPASS    -> ShooterConstants.FLYWHEEL_PASS_VELOCITY;
            case VISION_TRACKING -> variableVelocityRPS * ShooterConstants.FLYWHEEL_SPEED_RATIO;
            default -> 0.0;
        };

        double currentVelocity = flywheelVelocitySig.getValueAsDouble();
        return Math.abs(currentVelocity - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE_RPS;
    }

    public Command waitUntilReady() {
        return Commands.waitUntil(() -> atTargetVelocity() && atFlywheelTargetVelocity())
                .withTimeout(ShooterConstants.READY_TIMEOUT_SECONDS)
                .withName("WaitForShooterAndFlywheelReady");
    }
}   