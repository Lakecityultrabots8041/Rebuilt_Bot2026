package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final Servo upServo = new Servo(Constants.UpServoPort);
    private final Servo outServo = new Servo(Constants.UpServoPort);

    
    Servo UpServo = new Servo(Constants.UpServoPort);
    Servo OutServo = new Servo(Constants.OutServoPort);



   
 
    //=======================================================
    //Servo Commands Setup
    //=======================================================
    public void UpServoLock() {
        UpServo.setAngle(45);
        System.out.println("Did it Lock?");

        //return UpServoLock();
    }

    public void UpServoRelease() {
        UpServo.setAngle(0);
        System.out.println("Did it unlock?");

        //return UpServoRelease();
    }

    public void OutServoLock() {
        OutServo.setAngle(45);

        //return OutServoLock();
    }
    
    public void OutServoRelease() {
        OutServo.setAngle(0);

        //return OutServoRelease();
    }
    //rachet motor 
    
}
