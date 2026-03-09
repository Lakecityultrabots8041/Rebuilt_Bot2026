package frc.robot.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// Simulates the shooter flywheel so isFlywheelReady() works in sim.
public class FlywheelSimModel {

    private static final DCMotor MOTOR = DCMotor.getFalcon500(2);
    private static final double GEAR_RATIO = 1.0;
    
    // Change MOI if spin-up feels too fast or slow in sim
    private static final double MOI = 0.008;

    private final FlywheelSim flywheelSim;
    private final TalonFXSimState motor1SimState;
    private final TalonFXSimState motor2SimState;
    private double rotorPositionRotations = 0;

    public FlywheelSimModel(TalonFX motor1, TalonFX motor2) {
        flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(MOTOR, MOI, GEAR_RATIO),
            MOTOR
        );
        motor1SimState = motor1.getSimState();
        motor2SimState = motor2.getSimState();
    }

    public void update(double dtSeconds, double supplyVoltage) {
        motor1SimState.setSupplyVoltage(supplyVoltage);
        motor2SimState.setSupplyVoltage(supplyVoltage);

        double motorVoltage = motor1SimState.getMotorVoltage();
        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(dtSeconds);

        double velocityRPS = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        rotorPositionRotations += velocityRPS * dtSeconds;

        motor1SimState.setRawRotorPosition(rotorPositionRotations);
        motor1SimState.setRotorVelocity(velocityRPS);
        motor2SimState.setRawRotorPosition(rotorPositionRotations);
        motor2SimState.setRotorVelocity(velocityRPS);
    }
}
