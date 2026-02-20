package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
    private ShooterConstants() {}

    // ===== CAN =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    public static final int SHOOTER_MOTOR  = 5;
    public static final int FLYWHEEL_MOTOR = 6;

    // ===== Shooter speed presets (RPS) =====
    public static final double MAX_VELOCITY    = 90.0;
    public static final double REV_VELOCITY    = 65.0;
    public static final double EJECT_VELOCITY  = -5.0;
    public static final double IDLE_VELOCITY   =  0.0;

    // ===== Flywheel speed presets (RPS) =====
    // Flywheel is the top wheel — runs faster than the shooter to impart backspin and arc.
    // Tune FLYWHEEL_MAX_VELOCITY on the robot. See docs/SHOOTER_TUNING.md.
    public static final double FLYWHEEL_MAX_VELOCITY   = 105.0;
    public static final double FLYWHEEL_REV_VELOCITY   =  75.0;
    public static final double FLYWHEEL_IDLE_VELOCITY  =   0.0;

    // ===== Passing speed presets (RPS) =====
    // Higher velocity, lower flywheel ratio → flatter, longer-range trajectory.
    // Tune on the field. See docs/SHOOTER_TUNING.md.
    public static final double PASS_VELOCITY          = 100.0;
    public static final double FLYWHEEL_PASS_VELOCITY = 110.0;

    /**
     * Ratio applied to the shooter's variable (distance-based) velocity to get flywheel velocity.
     * Keeps the two motors proportional without needing a second distance table.
     * Tune this on the robot — a ratio of 1.15 means flywheel spins 15% faster than shooter.
     */
    public static final double FLYWHEEL_SPEED_RATIO = 1.15;

    // ===== Shooter PID =====
    public static final double kP = 3.0;
    public static final double kV = 0.15;
    public static final double kS = 0.25;

    // ===== Flywheel PID =====
    // Start identical to shooter — tune independently if flywheel has different inertia or friction.
    public static final double FLYWHEEL_kP = 3.0;
    public static final double FLYWHEEL_kV = 0.15;
    public static final double FLYWHEEL_kS = 0.25;

    // ===== Tolerances =====
    public static final double VELOCITY_TOLERANCE_RPS  = 2.0;
    public static final double READY_TIMEOUT_SECONDS   = 2.0;

    // ===== Variable distance velocity table =====
    // Shooter motor only. Flywheel scales by FLYWHEEL_SPEED_RATIO automatically.
    private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
    private static final double[] VELOCITY_TABLE_RPS    = { 60,  70,   80,  85,  88,  90,  90};

    /** Returns interpolated shooter velocity (RPS) for a given distance in meters. */
    public static double getVelocityForDistance(double meters) {
        if (meters <= DISTANCE_TABLE_METERS[0]) {
            return VELOCITY_TABLE_RPS[0];
        }
        if (meters >= DISTANCE_TABLE_METERS[DISTANCE_TABLE_METERS.length - 1]) {
            return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];
        }
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
