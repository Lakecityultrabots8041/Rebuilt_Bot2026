package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
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
    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;

    // Roller uses DutyCycleOut (simple % power, no PID)
    private final DutyCycleOut intakeRequest = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicRequest;
    private final NeutralOut neutralRequest = new NeutralOut();

    // Pivot position signal, refreshed non-blocking in periodic()
    private final StatusSignal<Angle> pivotPositionSig;
    private final StatusSignal<Angle> pivotPositionSig2;

    private IntakeState intakeState = IntakeState.IDLE;
    private IntakeState lastIntakeState = null;
    private PivotState pivotState = PivotState.STOW;
    private PivotState lastPivotState = null;
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
        intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR);
        pivotMotor1 = new TalonFX(IntakeConstants.PIVOT_MOTOR1);
        pivotMotor2 = new TalonFX(IntakeConstants.PIVOT_MOTOR2);
        motionMagicRequest = new MotionMagicVoltage(0);

        // Intake roller config (12:1 gearbox)
        var intakeConfigs = new TalonFXConfiguration();
        intakeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        intakeConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT;
        intakeConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
        intakeConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
        intakeConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Slot0.kP = IntakeConstants.kP;
        pivotConfigs.Slot0.kI = IntakeConstants.kI;
        pivotConfigs.Slot0.kD = IntakeConstants.kD;
        pivotConfigs.Slot0.kG = IntakeConstants.kG;
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
        pivotConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;

        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        pivotConfigs.MotionMagic.MotionMagicJerk = IntakeConstants.JERK;

        intakeMotor.getConfigurator().apply(intakeConfigs);
        // Motor 1 (left side) goes counterclockwise for positive arm movement
        pivotMotor1.getConfigurator().apply(pivotConfigs);

        // Motor 2 (right side) goes clockwise for positive arm movement
        pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor2.getConfigurator().apply(pivotConfigs);

        // Arm must be physically at stow before powering on.
        // This tells the motor that the current position is stow (position 0).
        pivotMotor1.setPosition(IntakeConstants.STOW_POSITION);
        pivotMotor2.setPosition(IntakeConstants.STOW_POSITION);

        // Pivot position updates at 50 Hz (every 20 ms)
        pivotPositionSig = pivotMotor1.getPosition();
        pivotPositionSig2 = pivotMotor2.getPosition();
        pivotPositionSig.setUpdateFrequency(50);
        pivotPositionSig2.setUpdateFrequency(50);

        // Disable unused status frames to reduce CAN traffic.
        // Pivot keeps its position signal at 50 Hz, everything else gets disabled.
        // Intake roller has no signals we read, so all frames get disabled.
        pivotMotor1.optimizeBusUtilization();
        pivotMotor2.optimizeBusUtilization();
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

        // Both pivot motors run independent Motion Magic so each compensates for its own load
        if (pivotState != lastPivotState) {
            switch (pivotState) {
                case STOW -> {
                    pivotMotor1.setControl(motionMagicRequest.withPosition(IntakeConstants.STOW_POSITION));
                    pivotMotor2.setControl(motionMagicRequest.withPosition(IntakeConstants.STOW_POSITION));
                }
                case INTAKE -> {
                    pivotMotor1.setControl(motionMagicRequest.withPosition(IntakeConstants.INTAKE_POSITION));
                    pivotMotor2.setControl(motionMagicRequest.withPosition(IntakeConstants.INTAKE_POSITION));
                }
                case TRAVEL -> {
                    pivotMotor1.setControl(motionMagicRequest.withPosition(IntakeConstants.TRAVEL_POSITION));
                    pivotMotor2.setControl(motionMagicRequest.withPosition(IntakeConstants.TRAVEL_POSITION));
                }
                case IDLE -> {
                    pivotMotor1.setControl(neutralRequest);
                    pivotMotor2.setControl(neutralRequest);
                }
            }
            lastPivotState = pivotState;
        }

        // Non-blocking fetch of pivot positions
        BaseStatusSignal.waitForAll(0, pivotPositionSig, pivotPositionSig2);
        double currentPivotPosition = pivotPositionSig.getValueAsDouble();
        double followerPosition = pivotPositionSig2.getValueAsDouble();
        cachedPivotAtTarget = isPivotAtTarget(currentPivotPosition);
        SmartDashboard.putString("Intake/State", intakeState.toString());
        SmartDashboard.putString("Intake/Pivot State", pivotState.toString());
        SmartDashboard.putNumber("Intake/Pivot1 Position", currentPivotPosition);
        SmartDashboard.putNumber("Intake/Pivot2 Position", followerPosition);
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

    // Used by SimManager to feed physics back into the motor controllers
    public TalonFX getPivotMotor1() { return pivotMotor1; }
    public TalonFX getPivotMotor2() { return pivotMotor2; }
}
