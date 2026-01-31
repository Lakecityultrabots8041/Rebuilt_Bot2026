package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimberSubsystem {

    
    Servo UpServo = new Servo(Constants.UpServoPort);
    Servo OutServo = new Servo(Constants.OutServoPort);
   

    //=======================================================
    //Servo Commands Setup
    //=======================================================
    public double UpServoLock() {
        UpServo.setAngle(45);
        System.out.println("Did it Lock?");

        return UpServoLock();
    }

    public double UpServoRelease() {
        UpServo.setAngle(0);
        System.out.println("Did it unlock?");

        return UpServoRelease();
    }

    public double OutServoLock() {
        OutServo.setAngle(45);

        return OutServoLock();
    }
    
    public double OutServoRelease() {
        OutServo.setAngle(0);

        return OutServoRelease();
    }
    //rachet motor 
    
}
