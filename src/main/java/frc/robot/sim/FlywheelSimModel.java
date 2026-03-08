package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulates the shooter flywheel so isFlywheelReady() works in sim.
 * Without this, any command that waits for flywheel velocity hangs forever.
 *
 * Physics estimates below are "close enough" for testing command flow.
 * They do not need to match the real robot exactly.
 */
public class FlywheelSimModel {

    // Two Falcon 500s in parallel, no gearbox (1:1)
    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double GEAR_RATIO = 1.0;

    // Moment of inertia of the flywheel (kg*m^2).
    // 0.008 is a reasonable estimate for an FRC shooter wheel.
    // If spin-up feels too fast or slow in sim, change this number.
    private static final double MOI = 0.008;

    private final FlywheelSim flywheelSim;
    private final TalonFXSimState motor1SimState;
    private final TalonFXSimState motor2SimState;

    // Track rotor position by integrating velocity each loop
    private double rotorPositionRotations = 0;

    public FlywheelSimModel(TalonFX motor1, TalonFX motor2) {
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(MOTOR, MOI, GEAR_RATIO),
            MOTOR
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
        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(dtSeconds);

        // Get the simulated velocity (rad/s -> rotations per second)
        double velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        double velocityRPS = velocityRadPerSec / (2.0 * Math.PI);

        // Integrate position (needed for the encoder to report a changing value)
        rotorPositionRotations += velocityRPS * dtSeconds;

        // Write simulated values back to both motors
        motor1SimState.setRawRotorPosition(rotorPositionRotations);
        motor1SimState.setRotorVelocity(velocityRPS);
        motor2SimState.setRawRotorPosition(rotorPositionRotations);
        motor2SimState.setRotorVelocity(velocityRPS);
    }
}
