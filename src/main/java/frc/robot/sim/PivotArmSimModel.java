package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulates the intake pivot arm so isPivotAtTarget() works in sim.
 * Without this, any command that waits for the arm to reach a position hangs forever.
 *
 * The arm uses Motion Magic with gravity compensation (Arm_Cosine).
 * This sim models gravity so the arm behaves realistically when moving up vs down.
 *
 * Physics estimates below are "close enough" for testing command flow.
 * They do not need to match the real robot exactly.
 */
public class PivotArmSimModel {

    // Two Falcon 500s (leader + follower), estimated 16:1 gear reduction.
    // The arm moves ~3.9 rotor rotations from stow to intake, which at 16:1
    // is about 88 degrees of arm travel. Seems right for a pivot intake.
    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double GEAR_RATIO = 16.0;

    // Arm physical estimates (change if sim feels wrong)
    private static final double ARM_LENGTH_METERS = 0.5;   // ~20 inches
    private static final double ARM_MASS_KG = 3.0;         // ~6.6 lbs

    // Arm travel limits in radians (mechanism side, not motor side)
    // Stow is near horizontal (0 rad), intake is ~90 degrees down (-PI/2)
    private static final double MIN_ANGLE_RAD = -Math.PI / 2.0;
    private static final double MAX_ANGLE_RAD = 0.1; // small buffer past horizontal

    // Starting angle (stow position maps to about -0.3 rotor rotations)
    // -0.3 rotor rotations / 16 gear ratio = -0.01875 mechanism rotations = -6.75 degrees
    private static final double STARTING_ANGLE_RAD = Units.degreesToRadians(-6.75);

    private final SingleJointedArmSim armSim;
    private final TalonFXSimState motor1SimState;
    private final TalonFXSimState motor2SimState;

    public PivotArmSimModel(TalonFX motor1, TalonFX motor2) {
        armSim = new SingleJointedArmSim(
            MOTOR,
            GEAR_RATIO,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
            ARM_LENGTH_METERS,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            true,           // simulate gravity
            STARTING_ANGLE_RAD
        );

        motor1SimState = motor1.getSimState();
        motor2SimState = motor2.getSimState();
    }

    /** Call once per loop from SimManager.update(). */
    public void update(double dtSeconds, double supplyVoltage) {
        // Tell the motor controller what voltage the battery has
        motor1SimState.setSupplyVoltage(supplyVoltage);
        motor2SimState.setSupplyVoltage(supplyVoltage);

        // Read what voltage the motor controller wants to apply
        double motorVoltage = motor1SimState.getMotorVoltage();

        // Run the physics
        armSim.setInputVoltage(motorVoltage);
        armSim.update(dtSeconds);

        // Convert mechanism angle/velocity back to rotor (motor) units
        // Mechanism angle (radians) -> mechanism rotations -> rotor rotations
        double mechanismAngleRad = armSim.getAngleRads();
        double mechanismRotations = mechanismAngleRad / (2.0 * Math.PI);
        double rotorPosition = mechanismRotations * GEAR_RATIO;

        double mechanismVelRadPerSec = armSim.getVelocityRadPerSec();
        double mechanismVelRPS = mechanismVelRadPerSec / (2.0 * Math.PI);
        double rotorVelocity = mechanismVelRPS * GEAR_RATIO;

        // Write back to both motors (motor2 is a follower but sim doesn't handle that)
        motor1SimState.setRawRotorPosition(rotorPosition);
        motor1SimState.setRotorVelocity(rotorVelocity);
        motor2SimState.setRawRotorPosition(rotorPosition);
        motor2SimState.setRotorVelocity(rotorVelocity);
    }
}
