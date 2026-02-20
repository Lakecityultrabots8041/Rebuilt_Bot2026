package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.FuelDetectionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Rotates and drives toward the nearest detected fuel using the Hailo neural detector.
 * Uses robot-relative ChassisSpeeds — rotation centers the target, forward closes the gap.
 *
 * Ends when:
 *   - Target area exceeds FUEL_INTAKE_AREA_THRESHOLD (fuel is within intake range), OR
 *   - FUEL_APPROACH_TIMEOUT_SECONDS has elapsed
 *
 * Tune thresholds and gains in VisionConstants. See docs/HAILO_TRAINING.md.
 *
 * NOTE: Bind this command to a controller button in RobotContainer once a button is chosen.
 * Run it alongside intake controls, this command only moves the robot, it does not
 * trigger the intake motors or pivot. Use in parallel with IntakeCommands if desired.
 */
public class DriveToFuel extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final FuelDetectionSubsystem fuelDetection;

    private final double maxSpeedMps;
    private final ChassisSpeeds reusableSpeeds = new ChassisSpeeds();
    private final Timer timer = new Timer();

    public DriveToFuel(CommandSwerveDrivetrain drivetrain, FuelDetectionSubsystem fuelDetection) {
        this.drivetrain    = drivetrain;
        this.fuelDetection = fuelDetection;
        this.maxSpeedMps   = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        fuelDetection.activateFuelDetector();
        timer.restart();
        SmartDashboard.putString("Fuel/Status", "Searching...");
    }

    @Override
    public void execute() {
        if (!fuelDetection.hasTarget()) {
            stop();
            SmartDashboard.putString("Fuel/Status", "No target");
            return;
        }

        double tx   = fuelDetection.getTX();
        double area = fuelDetection.getArea();

        // Rotate to center the target — negative because positive TX means target is right,
        // so we turn right (positive omega in robot-relative is clockwise = right)
        double rotationFraction = MathUtil.clamp(
            tx * VisionConstants.FUEL_ROTATION_GAIN,
            -VisionConstants.MAX_ROTATION_SPEED,
             VisionConstants.MAX_ROTATION_SPEED);

        // Drive forward at a fixed fraction of max speed while target is visible
        double forwardFraction = VisionConstants.FUEL_APPROACH_SPEED_FRACTION;

        reusableSpeeds.vxMetersPerSecond  = forwardFraction * maxSpeedMps;
        reusableSpeeds.vyMetersPerSecond  = 0;
        reusableSpeeds.omegaRadiansPerSecond = rotationFraction * maxSpeedMps;
        drivetrain.driveRobotRelative(reusableSpeeds);

        SmartDashboard.putString("Fuel/Status", "Approaching");
        SmartDashboard.putNumber("Fuel/TX Error", tx);
        SmartDashboard.putNumber("Fuel/Area", area);
    }

    @Override
    public void end(boolean interrupted) {
        stop();
        SmartDashboard.putString("Fuel/Status", interrupted ? "Interrupted" : "In range");
    }

    @Override
    public boolean isFinished() {
        return fuelDetection.isCloseEnoughToIntake()
            || timer.hasElapsed(VisionConstants.FUEL_APPROACH_TIMEOUT_SECONDS);
    }

    private void stop() {
        reusableSpeeds.vxMetersPerSecond  = 0;
        reusableSpeeds.vyMetersPerSecond  = 0;
        reusableSpeeds.omegaRadiansPerSecond = 0;
        drivetrain.driveRobotRelative(reusableSpeeds);
    }
}
