package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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

    private final String limelightName;
    private final boolean isRearFacing;
    private final String dashboardPrefix;

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
    private VisionConstants.VisionPoseConsumer visionPoseConsumer = null;
    private Supplier<Double> spinRateSupplier = null;

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

    /**
     * @param cameraName  NetworkTables name, e.g. "limelight-april" or "limelight-climber"
     * @param rearFacing  true if the camera faces the back of the robot
     */

     
    public LimelightSubsystem(String cameraName, boolean rearFacing) {
        this.limelightName = cameraName;
        this.isRearFacing = rearFacing;
        this.dashboardPrefix = cameraName + "/";
        limelightTable = NetworkTableInstance.getDefault().getTable(cameraName);
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

    /** Where to send accepted vision poses. Set from RobotContainer. */
    public void setVisionPoseConsumer(VisionConstants.VisionPoseConsumer consumer) {
        this.visionPoseConsumer = consumer;
    }

    /** How fast the robot is spinning (rad/s). Used to skip vision during fast turns. */
    public void setSpinRateSupplier(Supplier<Double> supplier) {
        this.spinRateSupplier = supplier;
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

    /** True if this camera faces the rear of the robot. */
    public boolean isRearFacing() {
        return isRearFacing;
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
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (result != null && result.tagCount > 0) {
            return result.pose;
        }
        return null;
    }

    public double getMegaTag2Timestamp() {
        if (isSimulation) return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
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
            LimelightHelpers.SetRobotOrientation_NoFlush(limelightName,
                robotPoseSupplier != null ? robotPoseSupplier.get().getRotation().getDegrees() : 0,
                0, 0, 0, 0, 0);
            cachedMegaTag2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            updateMegaTag2Cache();
            updateVisionFusion();
        }

        // Dashboard -- each camera gets its own prefix (e.g. "limelight-april/" or "limelight-climber/")
        SmartDashboard.putBoolean(dashboardPrefix + "Has Target", hasValidTarget());
        SmartDashboard.putNumber(dashboardPrefix + "TX (deg)", getHorizontalOffset());
        SmartDashboard.putNumber(dashboardPrefix + "TY (deg)", getVerticalOffset());
        SmartDashboard.putNumber(dashboardPrefix + "Area", getTargetArea());
        SmartDashboard.putNumber(dashboardPrefix + "AprilTag ID", getAprilTagID());
        SmartDashboard.putBoolean(dashboardPrefix + "MegaTag2 Available", hasMegaTag2Data());
        SmartDashboard.putBoolean(dashboardPrefix + "Sim Mode", isSimulation);

        SmartDashboard.putNumber(dashboardPrefix + "Distance (in)", getDistanceInchesForDisplay());
        SmartDashboard.putNumber(dashboardPrefix + "Distance (m)", getDistanceMeters());
    }

    private void updateMegaTag2Cache() {
        try {
            double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
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

    // Filters bad vision data, then sends good data to the pose estimator.
    // Accuracy scales automatically: more tags + closer = trust camera more.
    private void updateVisionFusion() {
        if (visionPoseConsumer == null) return;
        if (!hasValidTarget()) return;
        if (robotPoseSupplier == null) return;

        LimelightHelpers.PoseEstimate result = cachedMegaTag2Estimate;
        if (result == null) return;

        if (result.tagCount == 0) {
            SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "No tags");
            return;
        }
        if (result.avgTagArea < VisionConstants.MIN_TARGET_AREA) {
            SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "Area too small");
            return;
        }

        double x = result.pose.getX();
        double y = result.pose.getY();
        if (x < 0 || x > 17.0 || y < 0 || y > 9.0) {
            SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "Off field");
            return;
        }

        // Skip during fast spins. Camera images blur and give bad poses.
        if (spinRateSupplier != null) {
            double spin = Math.abs(spinRateSupplier.get());
            if (spin > VisionConstants.VISION_MAX_SPIN) {
                SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "Spinning too fast");
                return;
            }
        }

        // Skip if vision says we teleported. One bad frame shouldn't move the robot.
        Pose2d currentPose = robotPoseSupplier.get();
        double jump = currentPose.getTranslation().getDistance(result.pose.getTranslation());
        if (jump > VisionConstants.VISION_MAX_JUMP) {
            SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "Jumped " + String.format("%.1fm", jump));
            return;
        }

        // How accurate is this measurement? More tags + closer = better.
        // Start with VISION_ACCURACY, divide by tags squared, scale up by distance.
        double howFarOff = VisionConstants.VISION_ACCURACY / (result.tagCount * result.tagCount);
        howFarOff *= (1.0 + (result.avgTagDist * result.avgTagDist / 30.0));
        howFarOff = Math.max(0.1, Math.min(5.0, howFarOff));

        // CTRE needs three numbers: [x uncertainty, y uncertainty, rotation uncertainty]
        // We use the same value for x and y. Rotation is 999 = "ignore it, gyro is better."
        var poseUncertainty = VecBuilder.fill(howFarOff, howFarOff, 999.0);

        visionPoseConsumer.accept(result.pose, result.timestampSeconds, poseUncertainty);

        SmartDashboard.putString(dashboardPrefix + "Vision/Reject", "Accepted");
        SmartDashboard.putNumber(dashboardPrefix + "Vision/HowFarOff", howFarOff);
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
        // Rear-facing camera looks behind the robot
        if (isRearFacing) {
            robotHeading += Math.PI;
        }

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