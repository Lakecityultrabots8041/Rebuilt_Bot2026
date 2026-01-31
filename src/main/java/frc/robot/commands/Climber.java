package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;

public class Climber {

    Servo Servo = new Servo(1); // Defines servo, change channel to whatever its plugged into

    public Climber() {
        Servo.setAngle(80);
    }
    //Ways to set servo position
    /* 
    exampleServo.set(.5); // 0 to 1 range, can use decimals
    exampleServo.setAngle(75); //sets servo using degrees specifically, 0 to 180
*/
}