package frc.robot.subsystems.intake;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")



public class IntakeSubsystems extends SubsystemBase {
    
private final TalonFXConfiguration intakeConfigs = new TalonFXConfiguration();
private final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

private final TalonFX intakeMotor;
private final TalonFX pivotMotor;
private final VelocityTorqueCurrentFOC velocityRequest;

private PivotState pivotSate = PivotState.DOWN;
private IntakeState intakeState = IntakeState.IDLE;
private IntakeState lastIntakeState = null;
private PivotState lastPivotState = null;

public enum PivotState {
    UP,
    DOWN,
    IDLE
}

public enum IntakeState {
    INTAKING,
    EJECTING,
    IDLE
}

public boolean isUP() {
    return pivotSate == PivotState.UP;
}

public IntakeSubsystems() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR, IntakeConstants.CANIVORE_NAME);
    pivotMotor = new TalonFX(IntakeConstants.PIVOT_MOTOR, IntakeConstants.CANIVORE_NAME);
    velocityRequest = new VelocityTorqueCurrentFOC(0);

    var intakecConfigs = new TalonFXConfiguration();
    intakecConfigs.Slot0.kP = 0.1; //Change later in motion magic when we tune the motors
    intakecConfigs.Slot0.kV = 0.1; //Change later in motion magic when we tune the motors
    intakecConfigs.Slot0.kS = 0.1; //Change later in motion magic when we tune the motors

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

    intakeMotor.getConfigurator().apply(intakecConfigs);
    pivotMotor.getConfigurator().apply(pivotConfigs);
}

@Override
public void periodic() {
    // Only send motor commands when state changes (not every loop)
    if (intakeState != lastIntakeState) {
        switch (intakeState) {
            case INTAKING -> setVelocity(IntakeConstants.INTAKE_VELOCITY); //Change when we find the speed we want it at
            case EJECTING -> setVelocity(IntakeConstants.EJECT_VELOCITY); //Change when we find the speed we want it at
            case IDLE -> setVelocity(IntakeConstants.IDLE_VELOCITY); //Change when we find the speed we want it at
        }
        lastIntakeState = intakeState;
    }

    if (pivotSate != lastPivotState) {
        switch (pivotSate) {
            case UP -> setVelocity(IntakeConstants.PIVOT_UP_VELOCITY); //Change when we find the speed we want it at
            case DOWN -> setVelocity(IntakeConstants.PIVOT_DOWN_VELOCITY); //Change when we find the speed we want it at
            case IDLE -> setVelocity(IntakeConstants.PIVOT_IDLE_VELOCITY); //Change when we find the speed we want it at
        }
        lastPivotState = pivotSate;
    }

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
public Command pivotUp() {
    return Commands.runOnce(() -> pivotSate = PivotState.UP)
        .withName("LiftingIntake");
}

public Command pivotDown() {
    return Commands.runOnce(() -> pivotSate = PivotState.DOWN)
        .withName("LoweringIntake");
}

// ===== HELPER METHODS =====
 private void setVelocity(double velocityRPS) {
        intakeMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        pivotMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    public IntakeState getState() {
        return intakeState;
    }

    public PivotState getPivotState() {
        return pivotSate;
    }

}


