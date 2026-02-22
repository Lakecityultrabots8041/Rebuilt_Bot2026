package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
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
 * Limelight 4 subsystem — handles real hardware via NetworkTables and
 * simulated vision from field layout. See vision.md for details.
 */
public class LimelightSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-april";

    // NetworkTables
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, tid;

    // MegaTag2 cache
    private double cachedDistanceMeters = 0;
    private double cachedLateralOffsetMeters = 0;
    private boolean megatag2Available = false;

    // Cached once per loop — prevents calling getBotPoseEstimate twice per periodic()
    private LimelightHelpers.PoseEstimate cachedMegaTag2Estimate = null;

    // Per-loop NT cache (read once in periodic, used everywhere)
    private boolean cachedHasTarget = false;
    private double cachedTx = 0;
    private double cachedTy = 0;
    private double cachedTa = 0;
    private int cachedTid = -1;

    // Vision fusion
    private BiConsumer<Pose2d, Double> visionMeasurementConsumer = null;

    // Simulation
    private final boolean isSimulation;
    private Supplier<Pose2d> robotPoseSupplier = null;
    private final AprilTagFieldLayout fieldLayout;

    private boolean simHasTarget = false;
    private double simTx = 0;
    private double simTy = 0;
    private double simArea = 0;
    private int simTagID = -1;
    private double simDistanceMeters = 0;
    private double simLateralOffsetMeters = 0;

    private static final double HORIZONTAL_FOV_DEG = 29.8;
    private static final double MAX_DETECTION_RANGE_METERS = 8.0;

    // Pre-computed tag positions for simulation — built once in constructor,
    // zero per-loop allocations in updateSimVision().
    private record SimTag(int id, double x, double y) {}
    private final List<SimTag> simTagCache = new ArrayList<>();

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");

        isSimulation = RobotBase.isSimulation();

        AprilTagFieldLayout tempLayout = null;
        try {
            tempLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            System.err.println("WARNING: Could not load 2026 field layout: " + e.getMessage());
        }
        fieldLayout = tempLayout;

        // Build the sim tag cache once so periodic() never allocates Optional or Pose objects.
        if (fieldLayout != null) {
            for (var tag : fieldLayout.getTags()) {
                fieldLayout.getTagPose(tag.ID).ifPresent(pose -> {
                    var t = pose.toPose2d().getTranslation();
                    simTagCache.add(new SimTag(tag.ID, t.getX(), t.getY()));
                });
            }
        }

        if (!isSimulation) {
            setPipeline(0);
        }
    }

    /** Required for sim and vision fusion — set from RobotContainer. */
    public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.robotPoseSupplier = poseSupplier;
    }

    /** Receives MegaTag2 poses for odometry fusion each loop. */
    public void setVisionMeasurementConsumer(BiConsumer<Pose2d, Double> consumer) {
        this.visionMeasurementConsumer = consumer;
    }

    // Limelight data — all return per-loop cached values (updated at top of periodic)
    public boolean hasValidTarget() {
        if (isSimulation) return simHasTarget;
        return cachedHasTarget;
    }

    /** Horizontal offset in degrees. Positive = target right. */
    public double getHorizontalOffset() {
        if (isSimulation) return simTx;
        return cachedTx;
    }

    public double getVerticalOffset() {
        if (isSimulation) return simTy;
        return cachedTy;
    }

    public double getTargetArea() {
        if (isSimulation) return simArea;
        return cachedTa;
    }

    public int getAprilTagID() {
        if (isSimulation) return simTagID;
        return cachedTid;
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

    // Distance — all in meters internally
    public double getDistanceMeters() {
        if (isSimulation) {
            return simHasTarget ? simDistanceMeters : -1;
        }
        if (!megatag2Available) return -1;
        return cachedDistanceMeters;
    }

    /** Lateral offset in meters. Positive = target right of center. */
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

    /** True if tracking an alliance hub tag within auto-aim range. */
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
            // Read all NT values once per loop — everything else uses cached fields
            cachedHasTarget = tv.getDouble(0) == 1;
            cachedTx = tx.getDouble(0.0);
            cachedTy = ty.getDouble(0.0);
            cachedTa = ta.getDouble(0.0);
            cachedTid = (int) tid.getInteger(-1);

            // Fetch MegaTag2 once per loop — shared by cache update and vision fusion
            LimelightHelpers.SetRobotOrientation_NoFlush(LIMELIGHT_NAME,
                robotPoseSupplier != null ? robotPoseSupplier.get().getRotation().getDegrees() : 0,
                0, 0, 0, 0, 0);
            cachedMegaTag2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
            updateMegaTag2Cache();
            updateVisionFusion();
        }

        // Dashboard
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight/TX (deg)", getHorizontalOffset());
        SmartDashboard.putNumber("Limelight/TY (deg)", getVerticalOffset());
        SmartDashboard.putNumber("Limelight/Area", getTargetArea());
        SmartDashboard.putNumber("Limelight/AprilTag ID", getAprilTagID());
        SmartDashboard.putBoolean("Limelight/MegaTag2 Available", hasMegaTag2Data());
        SmartDashboard.putBoolean("Limelight/Sim Mode", isSimulation);

        SmartDashboard.putNumber("Limelight/Distance (in)", getDistanceInchesForDisplay());
        SmartDashboard.putNumber("Limelight/Distance (m)", getDistanceMeters());
    }

    private void updateMegaTag2Cache() {
        try {
            double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
            if (targetPose != null && targetPose.length >= 3 && cachedHasTarget) {
                cachedLateralOffsetMeters = targetPose[0]; // X = lateral
                cachedDistanceMeters = targetPose[2];      // Z = forward
                megatag2Available = true;
            } else {
                megatag2Available = false;
            }
        } catch (Exception e) {
            megatag2Available = false;
        }
    }

    // Uses the estimate already fetched this loop — no second NT call
    private void updateVisionFusion() {
        if (visionMeasurementConsumer == null) return;
        if (!hasValidTarget()) return;
        if (robotPoseSupplier == null) return;

        LimelightHelpers.PoseEstimate result = cachedMegaTag2Estimate;
        if (result == null) return;
        if (result.tagCount == 0) return;
        if (result.avgTagArea < VisionConstants.MIN_TARGET_AREA) return;

        double x = result.pose.getX();
        double y = result.pose.getY();
        if (x < 0 || x > 17.0 || y < 0 || y > 9.0) return;

        visionMeasurementConsumer.accept(result.pose, result.timestampSeconds);
    }

    // Sim — finds closest visible tag from robot pose using pre-built cache.
    // No per-loop object allocations — Optional and Pose objects are built once at startup.
    private void updateSimVision() {
        if (robotPoseSupplier == null || simTagCache.isEmpty()) {
            simHasTarget = false;
            return;
        }

        Pose2d robotPose = robotPoseSupplier.get();
        if (robotPose == null) {
            simHasTarget = false;
            return;
        }

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();

        double bestDistance = Double.MAX_VALUE;
        int bestTagID = -1;
        double bestTx = 0;

        for (var tag : simTagCache) {
            double dx = tag.x() - robotX;
            double dy = tag.y() - robotY;
            double distance = Math.hypot(dx, dy);
            if (distance > MAX_DETECTION_RANGE_METERS) continue;

            double angleToTag = Math.atan2(dy, dx);
            double angleDiff = Math.toDegrees(angleToTag - robotHeading);
            while (angleDiff > 180) angleDiff -= 360;
            while (angleDiff < -180) angleDiff += 360;

            if (Math.abs(angleDiff) > HORIZONTAL_FOV_DEG) continue;

            if (distance < bestDistance) {
                bestDistance = distance;
                bestTagID = tag.id();
                bestTx = -angleDiff; // negate for Limelight TX convention
            }
        }

        if (bestTagID >= 0) {
            simHasTarget = true;
            simTagID = bestTagID;
            simTx = bestTx;
            simDistanceMeters = bestDistance;
            simLateralOffsetMeters = bestDistance * Math.sin(Math.toRadians(bestTx));
            simArea = Math.max(0.1, 10.0 / (bestDistance * bestDistance));
            simTy = 0;
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