package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
    private ShooterConstants() {} // Prevent instantiation

    // ===== CAN Bus =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    // ===== CAN IDs =====
    public static final int SHOOTER_MOTOR = 2;


    // Speed presets (rotations per second)
    public static final double MAX_VELOCITY = 90.0; 
    public static final double REV_VELOCITY = 65.0;  
    public static final double EJECT_VELOCITY = -5.0;
    public static final double IDLE_VELOCITY = 0.0;

    // PID Gains
    public static final double kP = 3.0;
    public static final double kV = 0.15;
    public static final double kS = 0.25;

    // Tolerances
    public static final double VELOCITY_TOLERANCE_RPS = 2.0;
    public static final double READY_TIMEOUT_SECONDS = 2.0;
}