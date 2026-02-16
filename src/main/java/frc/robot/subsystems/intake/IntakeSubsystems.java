package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystems extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;
    private final VelocityTorqueCurrentFOC velocityRequest;
    private final MotionMagicVoltage motionMagicRequest;

    private IntakeState intakeState = IntakeState.IDLE;
    private IntakeState lastIntakeState = null;
    private PivotState pivotState = PivotState.STOW;
    private PivotState lastPivotState = null;

    public enum PivotState {
        STOW,
        INTAKE,
        TRAVEL,
        IDLE
    }

    public enum IntakeState {
        INTAKING,
        EJECTING,
        IDLE
    }

    public IntakeSubsystems() {
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR, IntakeConstants.CANIVORE);
        pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR, IntakeConstants.CANIVORE);
        velocityRequest = new VelocityTorqueCurrentFOC(0);
        motionMagicRequest = new MotionMagicVoltage(0);

        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.Slot0.kP = 0.1;
        intakeConfigs.Slot0.kV = 0.1;
        intakeConfigs.Slot0.kS = 0.1;

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Slot0.kP = IntakeConstants.kP;
        pivotConfigs.Slot0.kI = IntakeConstants.kI;
        pivotConfigs.Slot0.kD = IntakeConstants.kD;
        pivotConfigs.Slot0.kS = IntakeConstants.kS;
        pivotConfigs.Slot0.kV = IntakeConstants.kV;
        pivotConfigs.Slot0.kA = IntakeConstants.kA;
        pivotConfigs.Slot0.kG = IntakeConstants.kG;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        pivotConfigs.MotionMagic.MotionMagicJerk = IntakeConstants.JERK;

        intakeMotor.getConfigurator().apply(intakeConfigs);
        pivotMotor.getConfigurator().apply(pivotConfigs);
    }

    @Override
    public void periodic() {
        // Intake motor — only send on state change
        if (intakeState != lastIntakeState) {
            switch (intakeState) {
                case INTAKING -> setIntakeVelocity(IntakeConstants.INTAKE_VELOCITY);
                case EJECTING -> setIntakeVelocity(IntakeConstants.EJECT_VELOCITY);
                case IDLE -> setIntakeVelocity(IntakeConstants.IDLE_VELOCITY);
            }
            lastIntakeState = intakeState;
        }

        // Pivot motor — Motion Magic positions, only send on state change
        if (pivotState != lastPivotState) {
            switch (pivotState) {
                case STOW -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.STOW_POSITION));
                case INTAKE -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.INTAKE_POSITION));
                case TRAVEL -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.TRAVEL_POSITION));
                case IDLE -> {}
            }
            lastPivotState = pivotState;
        }

        // Telemetry
        SmartDashboard.putString("Intake/State", intakeState.toString());
        SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
        SmartDashboard.putNumber("Intake/Pivot Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Intake/Pivot At Target", isPivotAtTarget());
    }

    // ===== INTAKE COMMANDS =====
    public Command intake() {
        return Commands.runOnce(() -> intakeState = IntakeState.INTAKING)
            .withName("RunningIntakeMotor");
    }

    public Command eject() {
        return Commands.runOnce(() -> intakeState = IntakeState.EJECTING)
            .withName("ReversingIntakeMotor");
    }

    public Command idle() {
        return Commands.runOnce(() -> intakeState = IntakeState.IDLE)
            .withName("StoppingIntakeMotor");
    }

    // ===== PIVOT COMMANDS =====
    public Command pivotToStow() {
        return Commands.runOnce(() -> pivotState = PivotState.STOW)
            .withName("PivotToStow");
    }

    public Command pivotToIntake() {
        return Commands.runOnce(() -> pivotState = PivotState.INTAKE)
            .withName("PivotToIntake");
    }

    public Command pivotToTravel() {
        return Commands.runOnce(() -> pivotState = PivotState.TRAVEL)
            .withName("PivotToTravel");
    }

    // ===== HELPER METHODS =====
    private void setIntakeVelocity(double velocityRPS) {
        intakeMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    public boolean isPivotAtTarget() {
        double targetPosition = switch (pivotState) {
            case STOW -> IntakeConstants.STOW_POSITION;
            case INTAKE -> IntakeConstants.INTAKE_POSITION;
            case TRAVEL -> IntakeConstants.TRAVEL_POSITION;
            case IDLE -> Double.NaN;
        };
        if (Double.isNaN(targetPosition)) return false;
        double currentPosition = pivotMotor.getPosition().getValueAsDouble();
        return Math.abs(currentPosition - targetPosition) < IntakeConstants.POSITION_TOLERANCE;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public PivotState getPivotState() {
        return pivotState;
    }
}
