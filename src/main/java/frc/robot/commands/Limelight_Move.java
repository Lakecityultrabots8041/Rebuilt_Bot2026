package frc.robot.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;

import static edu.wpi.first.units.Units.*;

/**
 * Vision alignment command that aligns to ANY visible AprilTag within
 * a specified group (e.g., all HUB tags, all TOWER tags, etc.)
 * 
 * The tag group is resolved each time the command starts, so it
 * automatically picks up the correct alliance from DriverStation.
 * 
 * Usage examples:
 *   // Auto-detects alliance at runtime:
 *   new Limelight_Move(drive, ll, VisionConstants::getHubTags, strafeSupplier)
 *   new Limelight_Move(drive, ll, VisionConstants::getTowerTags)
 *   
 *   // Hardcoded to specific alliance:
 *   new Limelight_Move(drive, ll, () -> VisionConstants.BLUE_HUB_TAGS, strafeSupplier)
 * 
 * ALL INTERNAL CALCULATIONS USE METERS.
 */
public class Limelight_Move extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final Supplier<Set<Integer>> tagGroupSupplier;
    private final DoubleSupplier driverStrafeSupplier;

    // Resolved at command start
    private Set<Integer> acceptedTagIDs;

    private final double maxSpeedMps;
    private final double maxAngularRateRps;

    private final Timer timer;
    private int alignedCount = 0;
    private double targetDistanceMeters;

    /**
     * Align to any tag in the group, no driver strafe (for autonomous).
     */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight,
                          Supplier<Set<Integer>> tagGroupSupplier) {
        this(drivetrain, limelight, tagGroupSupplier, () -> 0.0);
    }

    /**
     * Align to any tag in the group with driver strafe for arcing (teleop).
     * 
     * @param drivetrain         the swerve drivetrain
     * @param limelight          the Limelight subsystem
     * @param tagGroupSupplier   supplier that returns the Set of valid tag IDs
     *                           (e.g. VisionConstants::getHubTags for auto-alliance)
     * @param driverStrafeSupplier  driver's strafe input (-1 to 1, typically leftX)
     */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight,
                          Supplier<Set<Integer>> tagGroupSupplier, DoubleSupplier driverStrafeSupplier) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.tagGroupSupplier = tagGroupSupplier;
        this.driverStrafeSupplier = driverStrafeSupplier;

        maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRateRps = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        timer = new Timer();
        targetDistanceMeters = VisionConstants.DEFAULT_APRILTAG_DISTANCE;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Resolve the tag group NOW — picks up current alliance from DriverStation
        acceptedTagIDs = tagGroupSupplier.get();

        System.out.println("=== Vision Alignment Started ===");
        System.out.println("Alliance: " + (VisionConstants.isRedAlliance() ? "RED" : "BLUE"));
        System.out.println("Accepting tag IDs: " + acceptedTagIDs);

        limelight.setLEDMode(true);
        alignedCount = 0;
        timer.restart();

        // If we already see a valid tag, grab its distance
        if (limelight.hasValidTarget()) {
            int tagID = limelight.getAprilTagID();
            if (acceptedTagIDs.contains(tagID)) {
                targetDistanceMeters = VisionConstants.getAprilTagDistance(tagID);
                System.out.println("Aligning to tag " + tagID + " at " +
                                   Units.metersToInches(targetDistanceMeters) + " inches");
            }
        }
    }

    @Override
    public void execute() {
        // Check if we see ANY tag in our accepted group
        if (!limelight.hasValidTarget() || !acceptedTagIDs.contains(limelight.getAprilTagID())) {
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "Searching for tags...");
            alignedCount = 0;
            return;
        }

        // We see a valid tag in our group — align to it
        int tagID = limelight.getAprilTagID();
        targetDistanceMeters = VisionConstants.getAprilTagDistance(tagID);
        SmartDashboard.putString("Vision/Status", "Tracking tag " + tagID);

        double currentDistanceMeters = limelight.getDistanceMeters();
        if (currentDistanceMeters < 0) {
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "No distance data");
            alignedCount = 0;
            return;
        }

        double horizontalErrorDeg = limelight.getHorizontalOffset();
        double distanceErrorMeters = currentDistanceMeters - targetDistanceMeters;

        // ROTATION (AUTO) — positive TX = target right, need negative omega
        double rotationOutput = -horizontalErrorDeg * VisionConstants.ROTATION_GAIN;
        rotationOutput = MathUtil.clamp(rotationOutput,
            -VisionConstants.MAX_ROTATION_SPEED, VisionConstants.MAX_ROTATION_SPEED);
        double rotationVelocityRps = rotationOutput * maxAngularRateRps;

        // FORWARD (AUTO) — positive error = too far, drive forward
        double forwardOutput = distanceErrorMeters * VisionConstants.FORWARD_GAIN;
        forwardOutput = MathUtil.clamp(forwardOutput,
            -VisionConstants.MAX_FORWARD_SPEED, VisionConstants.MAX_FORWARD_SPEED);
        double forwardVelocityMps = forwardOutput * maxSpeedMps;

        // STRAFE (DRIVER) — allows arcing around target
        double driverStrafe = driverStrafeSupplier.getAsDouble();
        driverStrafe = Math.abs(driverStrafe) < 0.1 ? 0.0 : driverStrafe;
        double strafeVelocityMps = driverStrafe * maxSpeedMps * VisionConstants.MAX_DRIVER_STRAFE_SCALE;

        // Send to drivetrain (robot-relative)
        drivetrain.driveRobotRelative(new ChassisSpeeds(
            forwardVelocityMps,
            strafeVelocityMps,
            rotationVelocityRps
        ));

        // Debug output
        SmartDashboard.putNumber("Vision/Active Tag", tagID);
        SmartDashboard.putNumber("Vision/TX Error (deg)", horizontalErrorDeg);
        SmartDashboard.putNumber("Vision/Distance (in)", Units.metersToInches(currentDistanceMeters));
        SmartDashboard.putNumber("Vision/Target Distance (in)", Units.metersToInches(targetDistanceMeters));
        SmartDashboard.putNumber("Vision/Distance Error (in)", Units.metersToInches(distanceErrorMeters));
        SmartDashboard.putNumber("Vision/Rotation Output", rotationOutput);
        SmartDashboard.putNumber("Vision/Forward Output", forwardOutput);
        SmartDashboard.putNumber("Vision/Driver Strafe", driverStrafe);

        // Alignment check
        boolean rotationGood = Math.abs(horizontalErrorDeg) < VisionConstants.ALIGNMENT_TOLERANCE_DEGREES;
        boolean distanceGood = Math.abs(distanceErrorMeters) < VisionConstants.DISTANCE_TOLERANCE_METERS;

        SmartDashboard.putBoolean("Vision/Rotation Aligned", rotationGood);
        SmartDashboard.putBoolean("Vision/Distance Aligned", distanceGood);

        if (rotationGood && distanceGood) {
            alignedCount++;
        } else {
            alignedCount = 0;
        }
        SmartDashboard.putNumber("Vision/Aligned Loops", alignedCount);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        limelight.setLEDMode(false);

        if (interrupted) {
            System.out.println("=== Vision Alignment Interrupted ===");
        } else {
            System.out.println("=== Vision Alignment Complete! ===");
        }
        SmartDashboard.putString("Vision/Status", "Stopped");
    }

    @Override
    public boolean isFinished() {
        if (alignedCount >= VisionConstants.ALIGNED_LOOPS_REQUIRED) {
            System.out.println("Vision alignment achieved!");
            return true;
        }
        if (timer.hasElapsed(VisionConstants.ALIGNMENT_TIMEOUT_SECONDS)) {
            System.out.println("Vision alignment timed out!");
            return true;
        }
        return false;
    }
}