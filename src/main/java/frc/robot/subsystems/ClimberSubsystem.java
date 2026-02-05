package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Climber;

public class ClimberSubsystem extends SubsystemBase {

    //private final Servo liftServo = new Servo(Constants.ClimberConstants.LiftServoPort);
    //private final Servo pivotServo = new Servo(Constants.ClimberConstants.PivotServoPort);

    private double liftMaxPos = Constants.ClimberConstants.liftMaxPos;
    private double pivotMaxPos = Constants.ClimberConstants.pivotMaxPos;
    private double liftMinPos = Constants.ClimberConstants.liftMinPos;
    private double pivotMinPos = Constants.ClimberConstants.pivotMinPos;

    private final TalonFX liftMotor;
    private final TalonFX pivotMotor;
    private final Servo Lift_Servo;
    private final Servo Pivot_Servo;

    public boolean upLock = true;
    public boolean outLock = true;
    public boolean liftOver = false;
    public boolean outOver = false;
    public boolean liftUnder = false;
    public boolean outUnder = false;

    private double liftPos;
    private double outPos;
    
    //Servo Lift_Servo = new Servo(Constants.ClimberConstants.LiftServoPort);
    //Servo Pivot_Servo = new Servo(Constants.ClimberConstants.PivotServoPort);

    public ClimberSubsystem() {
        liftMotor = new TalonFX(Constants.ClimberConstants.Lift_Motor);
        pivotMotor = new TalonFX(Constants.ClimberConstants.Pivot_Motor);

        Lift_Servo = new Servo(Constants.ClimberConstants.LiftServoPort);
        Pivot_Servo = new Servo(Constants.ClimberConstants.PivotServoPort);

        //Config motor limits
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 25.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        //Brake when off
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        liftMotor.getConfigurator().apply(config);
        pivotMotor.getConfigurator().apply(config);

        //Safety(Start Locked)
        liftServoLock();
        pivotServoLock();
    }
   
 
    //=======================================================
    //Servo Commands Setup
    //=======================================================
    public void liftServoLock() {
        Lift_Servo.setAngle(Constants.ClimberConstants.LServo_Lock_Angle);
        System.out.println("Did it Lock?");

        upLock = true;
        //return LiftServoLock();
    }

    public void liftServoRelease() {
        Lift_Servo.setAngle(Constants.ClimberConstants.LServo_Release_Angle);
        System.out.println("Did it unlock?");

        upLock = false;
        //return LiftServoRelease();
    }

    public void pivotServoLock() {
        Pivot_Servo.setAngle(Constants.ClimberConstants.PServo_Lock_Angle);

        outLock= true;
        //return PivotServoLock();
    }
    
    public void pivotServoRelease() {
        Pivot_Servo.setAngle(Constants.ClimberConstants.PServo_Release_Angle);

        outLock = false;
        //return PivotServoRelease();
    }

    public void runLiftMotor(double speed) {
        if (upLock = true) {
            System.out.println("Can't run the lift motor, Lock is engaged");
            liftMotor.set(0);
            return;//stops the command so it won't continue down and move the motor anyway
        //} else if (liftOver = true){
            //System.out.println("Can't run lift motor, Climber too high");
            //liftMotor.set(0);
            //return;
        } else {
            liftMotor.set(speed);
        }

    }

    public void runPivotMotor(double speed) {
        if (outLock = true) {
            System.out.println("Can't run Pivot Motor, Lock is engaged");
            pivotMotor.set(0);
            return;
        //} else if (outOver = true) {
          //  System.out.println("Can't run pivot motor, pivoted to the max");
            //pivotMotor.set(0);
            //return;
        } else {
            pivotMotor.set(speed);
        }
    }

    public void stopHangMotors() {
        liftMotor.set(0);
        pivotMotor.set(0);
    }
    
    

    @Override

    public void periodic(){
        liftPos = liftMotor.getPosition().getValueAsDouble();
        outPos = pivotMotor.getPosition().getValueAsDouble();

        if (liftPos >= liftMaxPos) {
            liftOver = true;
        } else {
            liftOver = false;
        }

        if (outPos >= pivotMaxPos) {
            outOver = true;
        } else {
            outOver = false;
        }

        if (liftPos >= liftMinPos) {
            liftUnder = false;
        } else {
            liftUnder = true;
        }

        if (outPos >= pivotMinPos) {
            outUnder = false;
        } else {
            outUnder = true;
        }
    }
    
}
