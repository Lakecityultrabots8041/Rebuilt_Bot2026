package frc.robot.subsystems.climb;

import com.ctre.phoenix6.CANBus;

public class ClimberConstants {

    private ClimberConstants() {}

    // ==========Can Stuff==========
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);
    public static final int ClimbingMotor = 57; //TODO Change to fit what the motor on the new bot is

    //TODO Tune the below PID values
    // ==========PID Shenanigans==========
    public static final double Climb_KP = 10.0; 
    public static final double Climb_KI = 0.0;
    public static final double Climb_KD = 0.00005;
}