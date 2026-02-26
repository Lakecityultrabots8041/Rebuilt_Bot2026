package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystems extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;
    // Roller uses DutyCycleOut (simple % power, no PID)
    private final DutyCycleOut intakeRequest    = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest;
    private final NeutralOut neutralRequest = new NeutralOut();

    // Pivot position signal, refreshed non-blocking in periodic()
    private final StatusSignal<Angle> pivotPositionSig;

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
        motionMagicRequest = new MotionMagicVoltage(0);

        // Intake roller config (12:1 gearbox)
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT;
        intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT;

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

        // Arm must be physically at stow before powering on.
        // This tells the motor that the current position is stow (position 0).
        pivotMotor.setPosition(IntakeConstants.STOW_POSITION);

        // Pivot position updates at 50 Hz (every 20 ms)
        pivotPositionSig = pivotMotor.getPosition();
        pivotPositionSig.setUpdateFrequency(50);

        // Disable unused status frames to reduce CAN traffic.
        // Pivot keeps its position signal at 50 Hz, everything else gets disabled.
        // Intake roller has no signals we read, so all frames get disabled.
        pivotMotor.optimizeBusUtilization();
        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        // Intake motor only updates on state change
        if (intakeState != lastIntakeState) {
            switch (intakeState) {
                case INTAKING -> setIntakePower(IntakeConstants.INTAKE_POWER);
                case EJECTING -> setIntakePower(IntakeConstants.EJECT_POWER);
                case IDLE     -> setIntakePower(0.0);
            }
            lastIntakeState = intakeState;
        }

        // Pivot motor only updates on state change
        if (pivotState != lastPivotState) {
            switch (pivotState) {
                case STOW -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.STOW_POSITION));
                case INTAKE -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.INTAKE_POSITION));
                case TRAVEL -> pivotMotor.setControl(motionMagicRequest.withPosition(IntakeConstants.TRAVEL_POSITION));
                case IDLE -> pivotMotor.setControl(neutralRequest);
            }
            lastPivotState = pivotState;
        }

        // Non-blocking fetch of pivot position
        BaseStatusSignal.waitForAll(0, pivotPositionSig);
        double currentPivotPosition = pivotPositionSig.getValueAsDouble();
        cachedPivotAtTarget = isPivotAtTarget(currentPivotPosition);
        SmartDashboard.putString("Intake/State", intakeState.toString());
        SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
        SmartDashboard.putNumber("Intake/Pivot Position", currentPivotPosition);
        SmartDashboard.putBoolean("Intake/Pivot At Target", cachedPivotAtTarget);
    }

    // ===== INTAKE COMMANDS =====
    public Command intake() {
        return runOnce(() -> intakeState = IntakeState.INTAKING)
            .withName("RunningIntakeMotor");
    }

    public Command eject() {
        return runOnce(() -> intakeState = IntakeState.EJECTING)
            .withName("ReversingIntakeMotor");
    }

    public Command idle() {
        return runOnce(() -> intakeState = IntakeState.IDLE)
            .withName("StoppingIntakeMotor");
    }

    // ===== PIVOT COMMANDS =====
    public Command pivotToStow() {
        return runOnce(() -> pivotState = PivotState.STOW)
            .withName("PivotToStow");
    }

    public Command pivotToIntake() {
        return runOnce(() -> pivotState = PivotState.INTAKE)
            .withName("PivotToIntake");
    }

    public Command pivotToTravel() {
        return runOnce(() -> pivotState = PivotState.TRAVEL)
            .withName("PivotToTravel");
    }

    /** Release the pivot motor so the arm can bounce freely on the bumper. */
    public Command pivotIdle() {
        return runOnce(() -> pivotState = PivotState.IDLE)
            .withName("PivotIdle");
    }

    // ===== WAIT COMMANDS =====
    public Command waitUntilPivotAtTarget() {
        return Commands.waitUntil(() -> isPivotAtTarget(pivotPositionSig.getValueAsDouble()))
            .withTimeout(IntakeConstants.PIVOT_TIMEOUT_SECONDS)
            .withName("WaitForPivot");
    }

    // ===== HELPER METHODS =====
    private void setIntakePower(double power) {
        if (power == 0.0) {
            intakeMotor.setControl(neutralRequest);
        } else {
            intakeMotor.setControl(intakeRequest.withOutput(power));
        }
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
