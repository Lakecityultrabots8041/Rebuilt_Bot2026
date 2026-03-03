package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {

    private IntakeConstants() {}

    // ===== CAN IDs =====
    public static final int INTAKE_MOTOR = 3;
    public static final int PIVOT_MOTOR = 2;

    // Intake roller power (0.0 to 1.0)
    // DutyCycleOut, no PID. If balls slip, raise it. If too aggressive, lower it.
    public static final double INTAKE_POWER = 1.0;
    public static final double EJECT_POWER  = -1.0; // Negative = reverse

    // Pivot PID and feedforward (tune on robot)
    public static final double kP = 12.0;
    public static final double kI = 0;
    public static final double kD = 0.2;
    public static final double kS = 2.0;
    public static final double kV = 4.5;
    public static final double kA = 0.01;
    public static final double kG = 4.25;

    //Arm Cosine says that sensor positions must be at 0 when the arm is parallel to the ground, so...
    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile (tune for desired deployment speed)
    public static final int CRUISE_VELOCITY = 15;
    public static final int ACCELERATION = 25;
    public static final int JERK = 25;

    // Pivot preset positions (rotor rotations - tune on robot)
    public static final double STOW_POSITION = 0.0;
    public static final double INTAKE_POSITION = -23.3;
    public static final double TRAVEL_POSITION = -10.0;

    // Position tolerance for "at target" checks (rotations)
    public static final double POSITION_TOLERANCE = 0.1;

    // How long to wait for the pivot to reach its target before giving up
    public static final double PIVOT_TIMEOUT_SECONDS = 2.0;

    // Intake roller current limits (12:1 gearbox)
    // Keep these conservative to protect the motor and gearbox.
    // Stator = motor torque. Supply = battery draw.
    public static final double INTAKE_STATOR_CURRENT_LIMIT = 40.0;
    public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 30.0;

    // Pivot current limits
    // Stator = motor torque. Supply = battery draw.
    public static final double PIVOT_STATOR_CURRENT_LIMIT = 60.0;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 40.0;

    // Pivot soft limits (rotor rotations)
    //Forward = toward intake. Set just past INTAKE_POSITION.
    //Reverse = toward stow. Set at STOW_POSITION so it can't go past zero.
    public static final double SOFT_LIMIT_FORWARD = -23.8;
    public static final double SOFT_LIMIT_REVERSE = 0.0;
}
