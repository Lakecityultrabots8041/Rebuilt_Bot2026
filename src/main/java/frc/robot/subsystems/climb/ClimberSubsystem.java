package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private double liftMaxPos = ClimberConstants.LIFT_MAX_POS;
    private double pivotMaxPos = ClimberConstants.PIVOT_MAX_POS;
    private double liftMinPos = ClimberConstants.LIFT_MIN_POS;
    private double pivotMinPos = ClimberConstants.PIVOT_MIN_POS;

    private final TalonFX liftMotor;
    private final TalonFX pivotMotor;
    private final Servo Lift_Servo;
    private final Servo Pivot_Servo;

    public boolean upLock = false;
    public boolean outLock = false;
    public boolean liftOver = false;
    public boolean outOver = false;
    public boolean liftUnder = false;
    public boolean outUnder = false;
    
    private double liftPos;
    private double outPos;

    public enum Step {
        ZERO, ONE, TWO, THREE, FOUR, FIVE, CLUMB
    }
    
    private Step currentStep = Step.ZERO;  // Instance variable, not static set like you had it
    
    public boolean climbingUp = true;  // Track direction

    public ClimberSubsystem() {
        liftMotor = new TalonFX(ClimberConstants.LIFT_MOTOR);
        pivotMotor = new TalonFX(ClimberConstants.PIVOT_MOTOR);

        Lift_Servo = new Servo(1);
        Pivot_Servo = new Servo(2);

        //Config motor limits for 16:1 gearbox
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 35.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        //Brake when off
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        liftMotor.getConfigurator().apply(config);
        pivotMotor.getConfigurator().apply(config);

        //Safety(Start Locked) - commented out for testing
        //liftServoLock();
        //pivotServoLock();
    }
   
    //=======================================================
    // Servo Commands Setup
    //=======================================================
    public void liftServoLock() {
        Lift_Servo.setAngle(ClimberConstants.LSERVO_LOCK_ANGLE);
        System.out.println("Lift Locked");
        upLock = true;
    }

    public void liftServoRelease() {
        Lift_Servo.setAngle(ClimberConstants.LSERVO_RELEASE_ANGLE);
        System.out.println("Lift Unlocked");
        upLock = false;
    }

    public void pivotServoLock() {
        Pivot_Servo.setAngle(ClimberConstants.PSERVO_LOCK_ANGLE);
        System.out.println("Pivot Locked");
        outLock = true;
    }
    
    public void pivotServoRelease() {
        Pivot_Servo.setAngle(ClimberConstants.PSERVO_RELEASE_ANGLE);
        System.out.println("Pivot Unlocked");
        outLock = false;
    }

    public void runLiftMotor(double speed) {
        if (upLock == true) {
            System.out.println("Can't run the lift motor, Lock is engaged");
            liftMotor.set(0);
            return;
        } else {
            liftMotor.set(speed);
        }
    }

    public void runPivotMotor(double speed) {
        if (outLock == true) {
            System.out.println("Can't run Pivot Motor, Lock is engaged");
            pivotMotor.set(0);
            return;
        } else {
            pivotMotor.set(speed);
        }
    }

    public void stopHangMotors() {
        liftMotor.set(0);
        pivotMotor.set(0);
    }

    // Getter for current step
    public Step getCurrentStep() {
        return currentStep;
    }

    @Override
    public void periodic() {
        liftPos = liftMotor.getPosition().getValueAsDouble();
        outPos = pivotMotor.getPosition().getValueAsDouble();

        // SmartDashboard debugging
        SmartDashboard.putNumber("Lift Position", liftPos);
        SmartDashboard.putNumber("Pivot Position", outPos);
        SmartDashboard.putString("Climb Step", currentStep.toString());
        SmartDashboard.putBoolean("Climbing Up", climbingUp);
        SmartDashboard.putNumber("Lift Motor %", liftMotor.get());
        SmartDashboard.putBoolean("Lift Lock", upLock);
        SmartDashboard.putBoolean("Pivot Lock", outLock);

        // Update limit flags
        liftOver = (liftPos >= liftMaxPos);
        outOver = (outPos >= pivotMaxPos);
        liftUnder = (liftPos < liftMinPos);
        outUnder = (outPos < pivotMinPos);

        // State machine
        if (currentStep == Step.ZERO && liftOver) {
            currentStep = Step.ONE;
        }
        if (currentStep == Step.ONE && upLock) {
            currentStep = Step.TWO;
        }
        if (currentStep == Step.TWO && outOver) {
            currentStep = Step.THREE;
        }
        if (currentStep == Step.THREE && outLock) {
            currentStep = Step.FOUR;
        }
        // Limelight step when ready
        //if (currentStep == Step.FOUR && limelightSubsystem.onBar) {
        //    currentStep = Step.FIVE;
        //}

        
        if (currentStep == Step.FIVE && liftUnder) {
            currentStep = Step.CLUMB;
        }
        if (currentStep == Step.CLUMB) {
            climbingUp = !climbingUp;  // Toggle direction
            currentStep = Step.ZERO;    // Reset to start
        }
    }
}