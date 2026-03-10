package frc.robot.subsystems.shoot;

public final class ShooterConstants {
    private ShooterConstants() {}

    // ===== CAN IDs =====
    public static final int FEED_FLOOR = 5;
    public static final int FEED_CEILING = 7;
    public static final int LO4D3R = 8;         // 12:1 gearbox, pulls ball up into flywheel
    public static final int FLYWHEEL_MOTOR = 6;
    public static final int FLYWHEEL_MOTOR2 = 9;

    // ===== Feed roller power (floor + ceiling push ball toward flywheel) =====
    public static final double FEED_POWER  =  0.80;
    public static final double EJECT_POWER = -0.20;
    public static final double PASS_POWER  =  0.80;

    // ===== Lo4d3r power (upper roller, pulls ball up into flywheel) =====
    public static final double LO4D3R_POWER      = -0.80;
    public static final double LO4D3R_EJECT_POWER =  0.50;

    // ===== Flywheel speed presets (RPS = rotations per second) =====
    // Higher RPS = ball goes farther. See SHOOTER_TUNING.md for details.
    public static final double FLYWHEEL_READY_RPS = 43.0;
    public static final double FLYWHEEL_REV_RPS   = 30.0;
    public static final double FLYWHEEL_PASS_RPS  = 50.0;
    public static final double FLYWHEEL_IDLE_RPS  =  0.0;

    // ===== Flywheel PID (see SHOOTER_TUNING.md for how to tune) =====
    public static final double FLYWHEEL_kP = 3.0;
    public static final double FLYWHEEL_kV = 0.15;
    public static final double FLYWHEEL_kS = 0.25;

    // ===== Supply current limits (prevents brownouts, no stator limits) =====
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 45.0;
    public static final double FEED_SUPPLY_CURRENT_LIMIT     = 60.0;
    public static final double LO4D3R_SUPPLY_CURRENT_LIMIT   = 60.0;

    // ===== Tolerances =====
    public static final double FLYWHEEL_TOLERANCE_RPS = 2.0;
    public static final double READY_TIMEOUT_SECONDS  = 3.0;

    // ===== Distance-to-speed table (feet) =====
    // Limelight distance -> flywheel RPS. Interpolated automatically.
    // Tune on the field. See SHOOTER_TUNING.md for instructions.
    private static final double[] DISTANCE_TABLE_FEET = { 5.0,  6.5,   8.0,  10.0, 11.5, 13.0, 16.5};
    private static final double[] VELOCITY_TABLE_RPS  = {  35,   40,    45,    50,   55,   58,   60};

    // Limelight gives meters, we convert to feet for the lookup
    private static final double FEET_PER_METER = 3.28084;

    /** Picks flywheel RPS from the distance table. Input is meters from Limelight. */
    public static double getSpeedForDistance(double meters) {
        double feet = meters * FEET_PER_METER;
        if (feet <= DISTANCE_TABLE_FEET[0]) return VELOCITY_TABLE_RPS[0];
        if (feet >= DISTANCE_TABLE_FEET[DISTANCE_TABLE_FEET.length - 1])
            return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];

        for (int i = 0; i < DISTANCE_TABLE_FEET.length - 1; i++) {
            if (feet <= DISTANCE_TABLE_FEET[i + 1]) {
                double t = (feet - DISTANCE_TABLE_FEET[i])
                         / (DISTANCE_TABLE_FEET[i + 1] - DISTANCE_TABLE_FEET[i]);
                return VELOCITY_TABLE_RPS[i] + t * (VELOCITY_TABLE_RPS[i + 1] - VELOCITY_TABLE_RPS[i]);
            }
        }
        return VELOCITY_TABLE_RPS[VELOCITY_TABLE_RPS.length - 1];
    }
}
