package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight 4 subsystem with MegaTag2 support and SIMULATION.
 * 
 * 
 * 
 */

 
public class LimelightSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-april";

    // ===== REAL HARDWARE (NetworkTables) =====
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, tid;

    // ===== CACHED DATA (used by both real and sim) =====
    private double cachedDistanceMeters = 0;
    private double cachedLateralOffsetMeters = 0;
    private boolean megatag2Available = false;

    // ===== VISION FUSION =====
    private BiConsumer<Pose2d, Double> visionMeasurementConsumer = null;

    // ===== SIM SUPPORT =====
    private final boolean isSimulation;
    private Supplier<Pose2d> robotPoseSupplier = null;
    private final AprilTagFieldLayout fieldLayout;

    // Sim-generated values
    private boolean simHasTarget = false;
    private double simTx = 0;
    private double simTy = 0;
    private double simArea = 0;
    private int simTagID = -1;
    private double simDistanceMeters = 0;
    private double simLateralOffsetMeters = 0;

    // Camera FOV limits (Limelight 4 specs)
    private static final double HORIZONTAL_FOV_DEG = 29.8;  // half-width
    private static final double MAX_DETECTION_RANGE_METERS = 8.0;   

    /**
     * Creates a LimelightSubsystem.
     * Call setRobotPoseSupplier() after construction to enable sim mode.
     */
    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");

        isSimulation = RobotBase.isSimulation();

        // Load official 2026 Rebuilt field layout
        // This gives us the exact position of every AprilTag on the field
        AprilTagFieldLayout tempLayout = null;
        try {
            tempLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            System.err.println("WARNING: Could not load 2026 field layout: " + e.getMessage());
        }
        fieldLayout = tempLayout;

        if (!isSimulation) {
            setPipeline(0);
        }
    }

    /**
     * REQUIRED FOR SIM: Provide a supplier for the robot's current pose.
     * Call this from RobotContainer after creating the drivetrain.
     *
     * Example: limelight.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.robotPoseSupplier = poseSupplier;
    }

    /**
     * Sets the consumer that receives MegaTag2 vision poses for odometry fusion.
     * Called every loop when a valid MegaTag2 estimate is available.
     *
     * @param consumer accepts (Pose2d pose, Double timestampSeconds)
     */
    public void setVisionMeasurementConsumer(BiConsumer<Pose2d, Double> consumer) {
        this.visionMeasurementConsumer = consumer;
    }

    // =========================================================================
    // BASIC LIMELIGHT DATA (works in both real and sim mode)
    // =========================================================================

    public boolean hasValidTarget() {
        if (isSimulation) return simHasTarget;
        return tv.getDouble(0) == 1;
    }

    /** @return horizontal offset in DEGREES (positive = target is to the right) */
    public double getHorizontalOffset() {
        if (isSimulation) return simTx;
        return tx.getDouble(0.0);
    }

    /** @return vertical offset in DEGREES */
    public double getVerticalOffset() {
        if (isSimulation) return simTy;
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        if (isSimulation) return simArea;
        return ta.getDouble(0.0);
    }

    public int getAprilTagID() {
        if (isSimulation) return simTagID;
        return (int) tid.getInteger(-1);
    }

    public boolean isTargetID(int targetID) {
        return hasValidTarget() && getAprilTagID() == targetID;
    }

    public void setPipeline(int pipeline) {
        if (!isSimulation) {
            limelightTable.getEntry("pipeline").setNumber(pipeline);
        }
    }

    public void setLEDMode(boolean on) {
        if (!isSimulation) {
            limelightTable.getEntry("ledMode").setNumber(on ? 3 : 1);
        }
    }

    // =========================================================================
    // DISTANCE METHODS - ALL RETURN METERS
    // =========================================================================

    public double getDistanceMeters() {
        if (isSimulation) {
            return simHasTarget ? simDistanceMeters : -1;
        }
        if (!megatag2Available) return -1;
        return cachedDistanceMeters;
    }

    /**
     * @return lateral (left-right) offset to target in METERS.
     *         Positive = target is to the right of camera center.
     *         Returns 0 if no valid data.
     */
    public double getLateralOffsetMeters() {
        if (isSimulation) {
            return simHasTarget ? simLateralOffsetMeters : 0;
        }
        if (!megatag2Available) return 0;
        return cachedLateralOffsetMeters;
    }

    public double getDistanceInchesForDisplay() {
        double dist = getDistanceMeters();
        if (dist < 0) return -1;
        return Units.metersToInches(dist);
    }

    public boolean hasMegaTag2Data() {
        if (isSimulation) return simHasTarget;
        return megatag2Available;
    }

    /**
     * Returns true if the Limelight is currently tracking an alliance hub tag
     * within auto-aim range.
     */
    public boolean isTrackingHubTag() {
        if (!hasValidTarget()) return false;
        int tagId = getAprilTagID();
        if (!VisionConstants.getHubTags().contains(tagId)) return false;
        double distance = getDistanceMeters();
        return distance > 0 && distance < VisionConstants.AUTO_AIM_MAX_RANGE_METERS;
    }

    public Pose2d getRobotPose_MegaTag2() {
        if (isSimulation && robotPoseSupplier != null) {
            return robotPoseSupplier.get();
        }
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (result != null && result.tagCount > 0) {
            return result.pose;
        }
        return null;
    }

    public double getMegaTag2Timestamp() {
        if (isSimulation) return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (result != null && result.tagCount > 0) {
            return result.timestampSeconds;
        }
        return 0;
    }

   
    @Override
    public void periodic() {
        if (isSimulation) {
            updateSimVision();
        } else {
            updateMegaTag2Cache();
            updateVisionFusion();
        }

        // Dashboard — same for both modes
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight/TX (deg)", getHorizontalOffset());
        SmartDashboard.putNumber("Limelight/TY (deg)", getVerticalOffset());
        SmartDashboard.putNumber("Limelight/Area", getTargetArea());
        SmartDashboard.putNumber("Limelight/AprilTag ID", getAprilTagID());
        SmartDashboard.putBoolean("Limelight/MegaTag2 Available", hasMegaTag2Data());
        SmartDashboard.putBoolean("Limelight/Sim Mode", isSimulation);

        if (hasValidTarget()) {
            SmartDashboard.putNumber("Limelight/Distance (in)", getDistanceInchesForDisplay());
            SmartDashboard.putNumber("Limelight/Distance (m)", getDistanceMeters());
        }
    }

    // =========================================================================
    // REAL HARDWARE - MegaTag2 Cache
    // =========================================================================

    /**
     * Feeds MegaTag2 pose estimates into the CTRE Kalman filter via the consumer.
     * Must be called after updateMegaTag2Cache() so we know if a valid target exists.
     */
    private void updateVisionFusion() {
        if (visionMeasurementConsumer == null) return;
        if (!hasValidTarget()) return;
        if (robotPoseSupplier == null) return;

        // MegaTag2 requires the robot's current heading to be set first
        Pose2d currentPose = robotPoseSupplier.get();
        if (currentPose == null) return;

        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME,
            currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate result =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        // Filter: reject null, no tags, low area, off-field poses
        if (result == null) return;
        if (result.tagCount == 0) return;
        if (result.avgTagArea < VisionConstants.MIN_TARGET_AREA) return;

        // Reject poses clearly off the field (field is ~16.54m x ~8.07m)
        double x = result.pose.getX();
        double y = result.pose.getY();
        if (x < 0 || x > 17.0 || y < 0 || y > 9.0) return;

        visionMeasurementConsumer.accept(result.pose, result.timestampSeconds);
    }

    private void updateMegaTag2Cache() {
        try {
            double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
            if (targetPose != null && targetPose.length >= 3 && tv.getDouble(0) == 1) {
                cachedLateralOffsetMeters = targetPose[0]; // X = lateral offset (right-positive)
                cachedDistanceMeters = targetPose[2];      // Z = forward distance
                megatag2Available = true;
            } else {
                megatag2Available = false;
            }
        } catch (Exception e) {
            megatag2Available = false;
        }
    }

    // =========================================================================
    // SIMULATION - Fake vision from robot pose + field layout
    // =========================================================================

    /**
     * In sim: calculate what the camera WOULD see based on the robot's
     * simulated pose and the known AprilTag positions on the 2026 Rebuilt field.
     * 
     * Finds the closest visible tag within the camera's FOV and generates
     * realistic tx, distance, and area values.
     */
    private void updateSimVision() {
        // Can't sim without a pose supplier or field layout
        if (robotPoseSupplier == null || fieldLayout == null) {
            simHasTarget = false;
            return;
        }

        Pose2d robotPose = robotPoseSupplier.get();
        if (robotPose == null) {
            simHasTarget = false;
            return;
        }

        // Find the closest tag that's in front of the robot and within FOV
        double bestDistance = Double.MAX_VALUE;
        int bestTagID = -1;
        double bestTx = 0;

        for (var tag : fieldLayout.getTags()) {
            Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tag.ID);
            if (tagPoseOpt.isEmpty()) continue;

            Pose3d tagPose3d = tagPoseOpt.get();
            Translation2d tagPos = tagPose3d.toPose2d().getTranslation();
            Translation2d robotPos = robotPose.getTranslation();

            // Distance from robot to tag
            double distance = robotPos.getDistance(tagPos);

            // Skip if too far
            if (distance > MAX_DETECTION_RANGE_METERS) continue;

            // Calculate angle from robot's heading to the tag
            // Robot heading is robotPose.getRotation()
            double angleToTag = Math.atan2(
                tagPos.getY() - robotPos.getY(),
                tagPos.getX() - robotPos.getX()
            );
            double robotHeading = robotPose.getRotation().getRadians();

            // TX = angle difference (how far off-center the tag appears)
            // Positive TX = tag is to the right of camera center
            double angleDiff = Math.toDegrees(angleToTag - robotHeading);

            // Normalize to -180 to 180
            while (angleDiff > 180) angleDiff -= 360;
            while (angleDiff < -180) angleDiff += 360;

            // Check if within camera FOV
            if (Math.abs(angleDiff) > HORIZONTAL_FOV_DEG) continue;

            // Pick the closest visible tag
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTagID = tag.ID;
                // Negate because Limelight convention: positive TX = target right
                // but our angleDiff is positive when tag is to the left in FRC coords?
                bestTx = -angleDiff;
            }
        }

        if (bestTagID >= 0) {
            simHasTarget = true;
            simTagID = bestTagID;
            simTx = bestTx;
            simDistanceMeters = bestDistance;
            // Lateral offset: distance * sin(tx) gives meters to the right of center
            simLateralOffsetMeters = bestDistance * Math.sin(Math.toRadians(bestTx));
            // Fake area — larger when closer (rough approximation) ie a guess
            simArea = Math.max(0.1, 10.0 / (bestDistance * bestDistance));
            // Fake TY — not critical for your alignment but approximate it again
            simTy = 0; // Would need camera height math for accuracy
        } else {
            simHasTarget = false;
            simTagID = -1;
            simTx = 0;
            simTy = 0;
            simArea = 0;
            simDistanceMeters = 0;
            simLateralOffsetMeters = 0;
        }
    }
}