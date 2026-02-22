package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
    private ShooterConstants() {}

    // ===== CAN =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    public static final int ACT_FLOOR     = 5;
    public static final int ACT_CEILING   = 7;
    public static final int FLYWHEEL_MOTOR = 6;

    // ===== Feed roller power (0.0 to 1.0) =====
    // DutyCycleOut — no PID, just push the ball.
    // Tune FEED_POWER first. If the ball stalls, raise it. If it slams too hard, lower it.
    public static final double FEED_POWER  =  0.80;
    public static final double EJECT_POWER = -0.60; // Negative = reverse
    public static final double PASS_POWER  =  0.60;

    // ===== Flywheel speed presets (RPS) =====
    // Ball exit speed is controlled by flywheel RPS — higher = farther.
    // Feed power does not affect shot distance, only how fast the ball enters.
    public static final double FLYWHEEL_READY_RPS = 105.0;
    public static final double FLYWHEEL_REV_RPS   =  75.0; // Pre-spin before going to full speed
    public static final double FLYWHEEL_PASS_RPS  =  65.0;
    public static final double FLYWHEEL_IDLE_RPS  =   0.0;

    // ===== Flywheel PID =====
    // kV is the main tuning knob — it tells the motor how much voltage per RPS.
    // kS overcomes static friction to get the motor moving.
    // kP corrects for error once spinning.
    public static final double FLYWHEEL_kP = 3.0;
    public static final double FLYWHEEL_kV = 0.15;
    public static final double FLYWHEEL_kS = 0.25;

    // ===== Tolerances =====
    public static final double FLYWHEEL_TOLERANCE_RPS = 2.0;  // How close is "close enough"
    public static final double READY_TIMEOUT_SECONDS  = 3.0;  // Give up waiting after this long

    // ===== Variable distance velocity table =====
    // Flywheel speed is adjusted based on distance from the target (from Limelight).
    // Feed rollers always use FEED_POWER regardless of distance.
    private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
    private static final double[] VELOCITY_TABLE_RPS    = { 60,  70,   80,  85,  88,  90,  90};

    /** Returns interpolated flywheel velocity (RPS) for a given distance in meters. */
    public static double getVelocityForDistance(double meters) {
        if (meters <= DISTANCE_TABLE_METERS[0]) return VELOCITY_TABLE_RPS[0];
        if (meters >= DISTANCE_TABLE_METERS[DISTANCE_TABLE_METERS.length - 1])
            return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];

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
