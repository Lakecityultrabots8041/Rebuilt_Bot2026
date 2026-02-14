package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
    private ShooterConstants() {} // Prevent instantiation

    // ===== CAN Bus =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    // ===== CAN IDs =====
    public static final int SHOOTER_MOTOR = 2;
    public static final int FLYWHEEL_MOTOR = 3;


    // Speed presets (rotations per second)
    public static final double MAX_VELOCITY = 90.0; 
    public static final double REV_VELOCITY = 65.0;  
    public static final double EJECT_VELOCITY = -5.0;
    public static final double IDLE_VELOCITY = 0.0;

    // Flywheel speed presets (rotations per second)
    public static final double FLYWHEEL_MAX_VELOCITY = 90.0;
    public static final double FLYWHEEL_REV_VELOCITY = 65.0;
    public static final double FLYWHEEL_IDLE_VELOCITY = 0.0;

    // PID Gains
    public static final double kP = 3.0;
    public static final double kV = 0.15;
    public static final double kS = 0.25;

    // Tolerances
    public static final double VELOCITY_TOLERANCE_RPS = 2.0;
    public static final double READY_TIMEOUT_SECONDS = 2.0;

    // ===== Variable Distance Shooter RPM  =====
    private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
    private static final double[] VELOCITY_TABLE_RPS    = {60,  70,  80,   85,  88,  90,  90 };

    /**
     * Returns the interpolated shooter velocity (RPS) for a given distance in meters.
     * Clamps at boundaries — distances below 1.5m use 60 RPS, above 5.0m use 90 RPS.
     */
    public static double getVelocityForDistance(double meters) {
        // Clamp below minimum distance
        if (meters <= DISTANCE_TABLE_METERS[0]) {
            return VELOCITY_TABLE_RPS[0];
        }
        // Clamp above maximum distance
        if (meters >= DISTANCE_TABLE_METERS[DISTANCE_TABLE_METERS.length - 1]) {
            return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];
        }
        // Linear interpolation between table entries
        for (int i = 0; i < DISTANCE_TABLE_METERS.length - 1; i++) {
            if (meters <= DISTANCE_TABLE_METERS[i + 1]) {
                double t = (meters - DISTANCE_TABLE_METERS[i])
                         / (DISTANCE_TABLE_METERS[i + 1] - DISTANCE_TABLE_METERS[i]);
                return VELOCITY_TABLE_RPS[i] + t * (VELOCITY_TABLE_RPS[i + 1] - VELOCITY_TABLE_RPS[i]);
            }
        }
        return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];
    }
}