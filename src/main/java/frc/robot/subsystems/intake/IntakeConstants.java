package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {

    private IntakeConstants() {}

    // ===== CAN Bus =====
    public static final String CANIVORE_NAME = "Jeffery";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    // ===== CAN IDs =====
    public static final int INTAKE_MOTOR = 3;
    public static final int PIVOT_MOTOR = 2;

    // Intake roller speeds (rotations per second)
    public static final double INTAKE_VELOCITY = 30.0;
    public static final double EJECT_VELOCITY = -30.9;
    public static final double IDLE_VELOCITY = 0.0;

    // Pivot PID and feedforward (tune on robot)
    public static final double kP = 12.0;
    public static final double kI = 0;
    public static final double kD = 0.2;
    public static final double kS = 2.0;
    public static final double kV = 4.5;
    public static final double kA = 0.01;
    public static final double kG = 4.25;

    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

    // Motion Magic profile (tune for desired deployment speed)
    public static final int CRUISE_VELOCITY = 15;
    public static final int ACCELERATION = 20;
    public static final int JERK = 20;

    // Pivot preset positions (rotor rotations - tune on robot)
    public static final double STOW_POSITION = 0.0;
    public static final double INTAKE_POSITION = 5.0;
    public static final double TRAVEL_POSITION = 3.5;

    // Position tolerance for "at target" checks (rotations)
    public static final double POSITION_TOLERANCE = 0.1;

    // How long to wait for the pivot to reach its target before giving up
    public static final double PIVOT_TIMEOUT_SECONDS = 2.0;

    // Pivot current limits
    // Stator controls how hard the motor pushes. Keeps it from grinding against the bumper hard stop.
    // Supply limits current drawn from the battery.
    public static final double PIVOT_STATOR_CURRENT_LIMIT = 60.0;
    public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 35.0;

    // Pivot soft limits (rotor rotations - tune after measuring real positions on the robot)
    // Forward = toward intake (higher position number). Set just past INTAKE_POSITION so the
    // motor stops commanding into the bumper hard stop.
    // Reverse = toward stow. Set at STOW_POSITION so it can't go past where the encoder was seeded.
    public static final double SOFT_LIMIT_FORWARD = 5.5;
    public static final double SOFT_LIMIT_REVERSE = 0.0;
}
