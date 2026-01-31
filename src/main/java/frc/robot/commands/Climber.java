package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class Climber extends Command {

    private final ClimberSubsystem UpServo; 
    private final ClimberSubsystem OutServo;

   

    public Climber(ClimberSubsystem UpServo, ClimberSubsystem OutServo) {
        this.UpServo = UpServo;
        this.OutServo = OutServo;
    }

    public void initialize() {
        UpServo.UpServoLock();
        OutServo.OutServoLock();
        UpServo.UpServoRelease();
        OutServo.OutServoRelease();

    }

    public Command UpClimber() {
        //When we know what Motor is used for the up part of the climber, we need to put it here in between the other two
        UpServo.UpServoRelease();
        //Here "Motor" Up
        //OutServo.OutServoRelease();
        //Here is pivot motor
        //OutServo.OutServoLock();
        //Drive into bar
        //Here "Motor" Down
        //UpServo.UpServoLock();
        return UpClimber();
        
    }

    public Command DownClimber() {
        //UpServo.UpServoRelease();
        //Here "Motor" Up
        //Drive away from bar
        //OutServo.OutServoRelease();
        //Here is pivot motor
        //OutServo.OutServoLock();
        //Here "Motor" Down
        UpServo.UpServoLock();
        return DownClimber();
    }

    //Ways to set servo position
    /* 
    exampleServo.set(.5); // 0 to 1 range, can use decimals
    exampleServo.setAngle(75); //sets servo using degrees specifically, 0 to 180
*/
}