package frc.robot.subsystems.vision;

import java.util.Optional;
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
    private boolean megatag2Available = false;

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

    public double getDistanceInchesForDisplay() {
        double dist = getDistanceMeters();
        if (dist < 0) return -1;
        return Units.metersToInches(dist);
    }

    public boolean hasMegaTag2Data() {
        if (isSimulation) return simHasTarget;
        return megatag2Available;
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

    private void updateMegaTag2Cache() {
        try {
            double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
            if (targetPose != null && targetPose.length >= 3 && tv.getDouble(0) == 1) {
                cachedDistanceMeters = targetPose[2];
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
        }
    }
}