package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystems extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;
    //private final VelocityTorqueCurrentFOC velocityRequest;

    private final VelocityVoltage velocityRequest;


    private final MotionMagicVoltage motionMagicRequest;
    private final NeutralOut neutralRequest = new NeutralOut(); 



    private IntakeState intakeState = IntakeState.IDLE;
    private IntakeState lastIntakeState = null;
    private PivotState pivotState = PivotState.STOW;
    private PivotState lastPivotState = PivotState.STOW;
    private boolean cachedPivotAtTarget = false;

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
        //velocityRequest = new VelocityTorqueCurrentFOC(0);
        // In constructor:
        velocityRequest = new VelocityVoltage(0);
        motionMagicRequest = new MotionMagicVoltage(0);
        

        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.Slot0.kP = 5.5;   // was 5 
        intakeConfigs.Slot0.kV = 0.1;   // was 20
        intakeConfigs.Slot0.kS = 3.0;   // was 5
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Slot0.kP = IntakeConstants.kP;
        pivotConfigs.Slot0.kI = IntakeConstants.kI;
        pivotConfigs.Slot0.kD = IntakeConstants.kD;
        pivotConfigs.Slot0.kS = IntakeConstants.kS;
        pivotConfigs.Slot0.kV = IntakeConstants.kV;
        pivotConfigs.Slot0.kA = IntakeConstants.kA;
        pivotConfigs.Slot0.kG = IntakeConstants.kG;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT;

        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.SOFT_LIMIT_FORWARD;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.SOFT_LIMIT_REVERSE;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        pivotConfigs.MotionMagic.MotionMagicJerk = IntakeConstants.JERK;

        intakeMotor.getConfigurator().apply(intakeConfigs);
        pivotMotor.getConfigurator().apply(pivotConfigs);

        // Put the arm in starting config -> That way the pivot encoder is set to stow position everytime.
        // The arm must be physically placed at stow before powering on (required for legal start config).
        // This tells Motion Magic where the arm is so it can hold and move correctly from boot.
        pivotMotor.setPosition(IntakeConstants.STOW_POSITION);
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

        // Telemetry — read position once, use for both display and target check
        double currentPivotPosition = pivotMotor.getPosition().getValueAsDouble();
        cachedPivotAtTarget = isPivotAtTarget(currentPivotPosition);
        SmartDashboard.putString("Intake/State", intakeState.toString());
        SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
        SmartDashboard.putNumber("Intake/Pivot Position", currentPivotPosition);
        SmartDashboard.putBoolean("Intake/Pivot At Target", cachedPivotAtTarget);
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

    // ===== WAIT COMMANDS =====
    public Command waitUntilPivotAtTarget() {
        return Commands.waitUntil(() -> isPivotAtTarget(pivotMotor.getPosition().getValueAsDouble()))
            .withTimeout(IntakeConstants.PIVOT_TIMEOUT_SECONDS)
            .withName("WaitForPivot");
    }

    // ===== HELPER METHODS =====
    private void setIntakeVelocity(double velocityRPS) {
        if (velocityRPS == 0) {
            intakeMotor.setControl(neutralRequest);
        } else {
            intakeMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        }
        //intakeMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    public boolean isPivotAtTarget(double currentPosition) {
        double targetPosition = switch (pivotState) {
            case STOW -> IntakeConstants.STOW_POSITION;
            case INTAKE -> IntakeConstants.INTAKE_POSITION;
            case TRAVEL -> IntakeConstants.TRAVEL_POSITION;
            case IDLE -> Double.NaN;
        };
        if (Double.isNaN(targetPosition)) return false;
        return Math.abs(currentPosition - targetPosition) < IntakeConstants.POSITION_TOLERANCE;
    }

    public boolean isPivotAtTarget() {
        return cachedPivotAtTarget;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public PivotState getPivotState() {
        return pivotState;
    }
}
