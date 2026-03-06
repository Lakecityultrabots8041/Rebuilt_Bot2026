package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {

    private IntakeConstants() {}

    // ===== CAN IDs =====
    public static final int INTAKE_MOTOR = 3;
    public static final int PIVOT_MOTOR1 = 2;
    public static final int PIVOT_MOTOR2 = 19;

    // Intake roller power (0.0 to 1.0)
    // DutyCycleOut, no PID. If balls slip, raise it. If too aggressive, lower it.
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER  = -1.0; // Negative = reverse

    // Pivot PID and feedforward (tune on robot)
    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;

    //Arm Cosine says that sensor positions must be at 0 when the arm is parallel to the ground, so...
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile (tune for desired deployment speed)
    public static final int CRUISE_VELOCITY = 3;
    public static final int ACCELERATION = 6;
    public static final int JERK = 0;

    // Pivot preset positions (rotor rotations - tune on robot)
    public static final double STOW_POSITION = -0.3;
    public static final double INTAKE_POSITION = -4.2;
    public static final double TRAVEL_POSITION = -2.87;

    // Position tolerance for "at target" checks (rotations)
    public static final double POSITION_TOLERANCE = 0.05;

    // How long to wait for the pivot to reach its target before giving up
    public static final double PIVOT_TIMEOUT_SECONDS = 3.0;

    // Intake roller current limits (12:1 gearbox)
    // Keep these conservative to protect the motor and gearbox.
    // Stator = motor torque. Supply = battery draw.
    public static final double INTAKE_STATOR_CURRENT_LIMIT = 40.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 30.0;

    // Pivot current limits
    // Stator = motor torque. Supply = battery draw.
    public static final double PIVOT_STATOR_CURRENT_LIMIT = 60.0;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;
}
