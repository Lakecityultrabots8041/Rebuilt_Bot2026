package frc.robot.sim;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.RobotContainer;

/**
 * Owns all physics simulation models. Created once in Robot.simulationInit(),
 * updated every loop from Robot.simulationPeriodic().
 *
 * This keeps all sim code out of the real subsystem files.
 * If something breaks at competition, you never have to look in here.
 */
public class SimManager {

    private final FlywheelSimModel flywheel;
    private final PivotArmSimModel pivotArm;

    private static final double DT_SECONDS = 0.02; // 20ms robot loop

    public SimManager(RobotContainer robotContainer) {
        var shooter = robotContainer.getShooter();
        var intake = robotContainer.getIntake();

        flywheel = new FlywheelSimModel(
            shooter.getFlywheelMotor(),
            shooter.getFlywheelMotor2()
        );

        pivotArm = new PivotArmSimModel(
            intake.getPivotMotor1(),
            intake.getPivotMotor2()
        );
    }

    /** Call from Robot.simulationPeriodic() every loop. */
    public void update() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        flywheel.update(DT_SECONDS, batteryVoltage);
        pivotArm.update(DT_SECONDS, batteryVoltage);
    }
}
