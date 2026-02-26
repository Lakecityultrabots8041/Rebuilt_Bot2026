package frc.robot.subsystems.shoot;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
    private ShooterConstants() {}

    // ===== CAN =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    public static final int ACT_FLOOR     = 5;
    public static final int ACT_CEILING   = 7;
    public static final int Lo4d3r     = 8; // 12:1 gearbox reduction motor
    public static final int FLYWHEEL_MOTOR = 6;

    // ===== Feed roller power (-1.0 to 1.0) =====
    // DutyCycleOut uses -1.0 to 1.0 scale. 0.5 = 50% power, 1.0 = 100% power.
    // If the ball stalls, raise it. If it slams too hard, lower it.
    public static final double FEED_POWER  =  0.50;
    public static final double EJECT_POWER = -0.40; // Negative = reverse
    public static final double PASS_POWER  =  0.50;

    // ===== Lo4d3r power =====
    public static final double LO4D3R_POWER = 0.80;
    public static final double LO4D3R_EJECT_POWER = -0.50;

    // ===== Flywheel speed presets =====
    // RPS = Rotations Per Second. How fast the flywheel spins.
    // Higher RPS = ball goes farther. Feed power does not affect distance.
    public static final double FLYWHEEL_READY_RPS = 105.0;
    public static final double FLYWHEEL_REV_RPS   =  75.0; // Pre-spin before going to full speed
    public static final double FLYWHEEL_PASS_RPS  =  65.0;
    public static final double FLYWHEEL_IDLE_RPS  =   0.0;

    // ===== Flywheel PID =====
    // kV = main tuning knob. How much voltage per RPS of target speed.
    // kS = overcomes friction to start the motor spinning.
    // kP = corrects remaining error once spinning.
    public static final double FLYWHEEL_kP = 3.0;
    public static final double FLYWHEEL_kV = 0.15;
    public static final double FLYWHEEL_kS = 0.25;

    // ===== Current Limits =====
    // Stator = motor torque. Supply = battery draw.

    // Flywheel draws high current during spin-up, then drops at steady state.
    // Stator maxed to 120A for full torque during spin-up.
    public static final double FLYWHEEL_STATOR_CURRENT_LIMIT = 120.0;
    public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60.0;

    // Feed rollers (floor and ceiling, no gearbox reduction).
    // Stator maxed to 120A so they never stall pushing balls through.
    public static final double FEED_STATOR_CURRENT_LIMIT = 120.0;
    public static final double FEED_SUPPLY_CURRENT_LIMIT = 60.0;

    // Upper feed roller (12:1 gearbox, 4 belts). Needs high current
    // because the 4 belts add significant mechanical load.
    // Stator maxed to 120A for the same reason.
    public static final double UPPER_STATOR_CURRENT_LIMIT = 120.0;
    public static final double UPPER_SUPPLY_CURRENT_LIMIT = 60.0;

    // ===== Tolerances =====
    public static final double FLYWHEEL_TOLERANCE_RPS = 2.0;  // How close is "close enough"
    public static final double READY_TIMEOUT_SECONDS  = 3.0;  // Give up waiting after this long

    // ===== Variable distance velocity table =====
    // Flywheel speed is adjusted based on distance from the target (from Limelight).
    // Feed rollers always use FEED_POWER regardless of distance.
    private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
    private static final double[] VELOCITY_TABLE_RPS    = { 60,  70,   80,  85,  88,  90,  90};

    /** Returns flywheel speed (RPS) for a given distance in meters. */
    public static double getSpeedForDistance(double meters) {
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
