package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * Demo-only command: robot follows ANY visible AprilTag at a safe distance.
 * Hold someone walks around with a printed AprilTag and the robot trails them.
 *
 * Bind to a held button (whileTrue) so the robot stops when the button is released.
 * NOT for competition use, demo/outreach only.
 */
public class FollowTag_Demo extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;

    // --- Tuning knobs (all in one spot for easy tweaking) ---

    /** Target following distance in meters (~4 feet). */
    private static final double TARGET_DISTANCE_METERS = 1.2;

    // Proportional gains (same style as Limelight_Move)
    private static final double ROTATION_GAIN = 0.025;
    private static final double FORWARD_GAIN  = 0.6;
    private static final double STRAFE_GAIN   = 0.4;

    // Speed caps as fraction of max (kept low for safety in demos)
    private static final double MAX_FORWARD_SPEED  = 0.20;
    private static final double MAX_ROTATION_SPEED = 0.15;
    private static final double MAX_STRAFE_SPEED   = 0.15;

    private final double maxSpeedMps;
    private final double maxAngularRateRps;

    // Preallocated to avoid per-loop heap allocations
    private final ChassisSpeeds reusableSpeeds = new ChassisSpeeds();

    public FollowTag_Demo(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;

        maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRateRps = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        limelight.setLEDMode(true);
        SmartDashboard.putString("Demo/Status", "Following started");
    }

    @Override
    public void execute() {
        // No valid target -- stop and wait
        if (!limelight.hasValidTarget()) {
            stopDrivetrain();
            SmartDashboard.putString("Demo/Status", "Searching for tag...");
            return;
        }

        double currentDistance = limelight.getDistanceMeters();
        if (currentDistance < 0) {
            stopDrivetrain();
            SmartDashboard.putString("Demo/Status", "No distance data");
            return;
        }

        int tagID = limelight.getAprilTagID();
        double horizontalErrorDeg = limelight.getHorizontalOffset();
        double distanceError = currentDistance - TARGET_DISTANCE_METERS;
        double lateralOffset = limelight.getLateralOffsetMeters();

        // Rotation -- face the tag
        double rotationOutput = -horizontalErrorDeg * ROTATION_GAIN;
        rotationOutput = MathUtil.clamp(rotationOutput, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        // Forward/back -- maintain target distance
        double forwardOutput = distanceError * FORWARD_GAIN;
        forwardOutput = MathUtil.clamp(forwardOutput, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);

        // Strafe -- center on the tag
        double strafeOutput = lateralOffset * STRAFE_GAIN;
        strafeOutput = MathUtil.clamp(strafeOutput, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);

        reusableSpeeds.vxMetersPerSecond = forwardOutput * maxSpeedMps;
        reusableSpeeds.vyMetersPerSecond = strafeOutput * maxSpeedMps;
        reusableSpeeds.omegaRadiansPerSecond = rotationOutput * maxAngularRateRps;
        drivetrain.driveRobotRelative(reusableSpeeds);

        // Dashboard under Demo/ prefix
        SmartDashboard.putString("Demo/Status", "Tracking tag " + tagID);
        SmartDashboard.putNumber("Demo/Tag ID", tagID);
        SmartDashboard.putNumber("Demo/Distance (in)", Units.metersToInches(currentDistance));
        SmartDashboard.putNumber("Demo/Target Distance (in)", Units.metersToInches(TARGET_DISTANCE_METERS));
        SmartDashboard.putNumber("Demo/Distance Error (in)", Units.metersToInches(distanceError));
        SmartDashboard.putNumber("Demo/TX Error (deg)", horizontalErrorDeg);
        SmartDashboard.putNumber("Demo/Lateral Offset (in)", Units.metersToInches(lateralOffset));
        SmartDashboard.putNumber("Demo/Forward Output", forwardOutput);
        SmartDashboard.putNumber("Demo/Rotation Output", rotationOutput);
        SmartDashboard.putNumber("Demo/Strafe Output", strafeOutput);
    }

    @Override
    public void end(boolean interrupted) {
        stopDrivetrain();
        limelight.setLEDMode(false);
        SmartDashboard.putString("Demo/Status", "Stopped");
    }

    @Override
    public boolean isFinished() {
        // Never auto-finishes, runs until button is released
        return false;
    }

    private void stopDrivetrain() {
        reusableSpeeds.vxMetersPerSecond = 0;
        reusableSpeeds.vyMetersPerSecond = 0;
        reusableSpeeds.omegaRadiansPerSecond = 0;
        drivetrain.driveRobotRelative(reusableSpeeds);
    }
}
