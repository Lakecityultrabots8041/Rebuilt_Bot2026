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
 * Aligns to any visible AprilTag in a tag group. Tag group resolves at
 * command start so alliance detection works automatically. See vision.md.
 */
public class Limelight_Move extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final Supplier<Set<Integer>> tagGroupSupplier;
    private final DoubleSupplier driverStrafeSupplier;

    private Set<Integer> acceptedTagIDs;

    private final double maxSpeedMps;
    private final double maxAngularRateRps;

    private final Timer timer;
    private int alignedCount = 0;
    private double targetDistanceMeters;

    /** Autonomous — no driver strafe. */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight,
                          Supplier<Set<Integer>> tagGroupSupplier) {
        this(drivetrain, limelight, tagGroupSupplier, () -> 0.0);
    }

    /** Teleop — driver strafe input for arcing approaches. */
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
        acceptedTagIDs = tagGroupSupplier.get();

        System.out.println("=== Vision Alignment Started ===");
        System.out.println("Alliance: " + (VisionConstants.isRedAlliance() ? "RED" : "BLUE"));
        System.out.println("Accepting tag IDs: " + acceptedTagIDs);

        limelight.setLEDMode(true);
        alignedCount = 0;
        timer.restart();

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
        if (!limelight.hasValidTarget() || !acceptedTagIDs.contains(limelight.getAprilTagID())) {
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "Searching for tags...");
            alignedCount = 0;
            return;
        }

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

        // Rotation
        double rotationOutput = -horizontalErrorDeg * VisionConstants.ROTATION_GAIN;
        rotationOutput = MathUtil.clamp(rotationOutput,
            -VisionConstants.MAX_ROTATION_SPEED, VisionConstants.MAX_ROTATION_SPEED);
        double rotationVelocityRps = rotationOutput * maxAngularRateRps;

        // Forward/back
        double forwardOutput = distanceErrorMeters * VisionConstants.FORWARD_GAIN;
        forwardOutput = MathUtil.clamp(forwardOutput,
            -VisionConstants.MAX_FORWARD_SPEED, VisionConstants.MAX_FORWARD_SPEED);
        double forwardVelocityMps = forwardOutput * maxSpeedMps;

        // Strafe — driver input in teleop, auto-correction in auto
        double driverStrafe = driverStrafeSupplier.getAsDouble();
        driverStrafe = Math.abs(driverStrafe) < 0.1 ? 0.0 : driverStrafe;

        double strafeVelocityMps;
        double lateralOffsetMeters = limelight.getLateralOffsetMeters();

        if (Math.abs(driverStrafe) > 0.0) {
            strafeVelocityMps = driverStrafe * maxSpeedMps * VisionConstants.MAX_DRIVER_STRAFE_SCALE;
        } else {
            double strafeOutput = lateralOffsetMeters * VisionConstants.AUTO_STRAFE_GAIN;
            strafeOutput = MathUtil.clamp(strafeOutput,
                -VisionConstants.MAX_AUTO_STRAFE_SPEED, VisionConstants.MAX_AUTO_STRAFE_SPEED);
            strafeVelocityMps = strafeOutput * maxSpeedMps;
        }

        drivetrain.driveRobotRelative(new ChassisSpeeds(
            forwardVelocityMps,
            strafeVelocityMps,
            rotationVelocityRps
        ));

        SmartDashboard.putNumber("Vision/Active Tag", tagID);
        SmartDashboard.putNumber("Vision/TX Error (deg)", horizontalErrorDeg);
        SmartDashboard.putNumber("Vision/Distance (in)", Units.metersToInches(currentDistanceMeters));
        SmartDashboard.putNumber("Vision/Target Distance (in)", Units.metersToInches(targetDistanceMeters));
        SmartDashboard.putNumber("Vision/Distance Error (in)", Units.metersToInches(distanceErrorMeters));
        SmartDashboard.putNumber("Vision/Rotation Output", rotationOutput);
        SmartDashboard.putNumber("Vision/Forward Output", forwardOutput);
        SmartDashboard.putNumber("Vision/Lateral Offset (in)", Units.metersToInches(lateralOffsetMeters));
        SmartDashboard.putNumber("Vision/Strafe Output", strafeVelocityMps / maxSpeedMps);
        SmartDashboard.putNumber("Vision/Driver Strafe", driverStrafe);

        boolean rotationGood = Math.abs(horizontalErrorDeg) < VisionConstants.ALIGNMENT_TOLERANCE_DEGREES;
        boolean distanceGood = Math.abs(distanceErrorMeters) < VisionConstants.DISTANCE_TOLERANCE_METERS;
        boolean strafeGood = Math.abs(lateralOffsetMeters) < VisionConstants.STRAFE_TOLERANCE_METERS;

        SmartDashboard.putBoolean("Vision/Rotation Aligned", rotationGood);
        SmartDashboard.putBoolean("Vision/Distance Aligned", distanceGood);
        SmartDashboard.putBoolean("Vision/Strafe Aligned", strafeGood);

        if (rotationGood && distanceGood && strafeGood) {
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