package frc.robot.subsystems.climb;

/**
 * Constants for the climbing subsystem.
 * ALL POSITIONS ARE IN MOTOR ROTATIONS (through 16:1 gearbox).
 * 
 * TODO: Measure actual position limits on the real robot!
 */

 
public final class ClimberConstants {

    private ClimberConstants() {}

    // ===== CAN IDs =====
    public static final int LIFT_MOTOR = 3;
    public static final int PIVOT_MOTOR = 87; // TODO: Fix port number when we get the motor

    // ===== PWM Ports =====
    public static final int LIFT_SERVO_PORT = 1;
    public static final int PIVOT_SERVO_PORT = 2;

    // ===== Servo Angles =====
    public static final double LSERVO_LOCK_ANGLE = 30.0;
    public static final double LSERVO_RELEASE_ANGLE = 0.0;
    public static final double PSERVO_LOCK_ANGLE = 10.0;
    public static final double PSERVO_RELEASE_ANGLE = 0.0;

    // ===== Motor Speed Limits =====
    public static final double LIFT_MOTOR_MAX = 0.5;
    public static final double PIVOT_MOTOR_MAX = 0.5;

    // ===== Current Limits (amps) =====
    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 35.0;

    // ===== Position Limits (motor rotations) =====
    // TODO: These are placeholder values — measure on real robot!
    public static final double LIFT_MAX_POS = 50.0;
    public static final double LIFT_MIN_POS = 0.0;
    public static final double PIVOT_MAX_POS = 50.0;
    public static final double PIVOT_MIN_POS = 0.0;

    // ===== Climb Speeds =====
    public static final double CLIMB_SPEED = 0.6;
    public static final double FINAL_CLIMB_SPEED = 0.5;
    public static final double SERVO_DELAY_SECONDS = 0.5;
}