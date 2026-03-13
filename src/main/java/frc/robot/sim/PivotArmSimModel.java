package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Simulates the intake pivot arm so isPivotAtTarget() works in sim.
public class PivotArmSimModel {

    private static final DCMotor MOTOR = DCMotor.getKrakenX60(2);
    // Change gear ratio if arm travel doesn't match real robot
    private static final double GEAR_RATIO = 12.0;
    // Change these if arm movement feels wrong in sim
    private static final double ARM_LENGTH_METERS = 0.5;
    private static final double ARM_MASS_KG = 3.0;

    // Arm travels ~1.53 rad from stow to intake through the 16:1 ratio
    private static final double MIN_ANGLE_RAD = -0.5;
    private static final double MAX_ANGLE_RAD = 2.0;
    // Must start at 0 because setPosition(-0.3) in the subsystem adds an offset
    private static final double STARTING_ANGLE_RAD = 0;

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
            true, // gravity
            STARTING_ANGLE_RAD
        );

        motor1SimState = motor1.getSimState();
        motor2SimState = motor2.getSimState();
    }

    public void update(double dtSeconds, double supplyVoltage) {
        motor1SimState.setSupplyVoltage(supplyVoltage);
        motor2SimState.setSupplyVoltage(supplyVoltage);

        double motorVoltage = motor1SimState.getMotorVoltage();
        armSim.setInputVoltage(motorVoltage);
        armSim.update(dtSeconds);

        // Convert mechanism angle back to rotor rotations
        double rotorPosition = armSim.getAngleRads() / (2.0 * Math.PI) * GEAR_RATIO;
        double rotorVelocity = armSim.getVelocityRadPerSec() / (2.0 * Math.PI) * GEAR_RATIO;

        motor1SimState.setRawRotorPosition(rotorPosition);
        motor1SimState.setRotorVelocity(rotorVelocity);
        // Motor 2 is mounted opposite, raw rotor spins the other way
        motor2SimState.setRawRotorPosition(-rotorPosition);
        motor2SimState.setRotorVelocity(-rotorVelocity);
    }
}
