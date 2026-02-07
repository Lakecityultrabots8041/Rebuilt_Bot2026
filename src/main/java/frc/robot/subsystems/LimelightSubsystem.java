package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;  

/**
 * Subsystem for Limelight 4 camera with MegaTag2 support
 * 
 * ALL DISTANCE METHODS RETURN METERS unless explicitly named otherwise.
 * 
 * TO USE MEGATAG2:
 * 1. Download LimelightHelpers.java from:
 *    https://github.com/LimelightVision/limelern-examples/blob/main/java-examples/LimelightHelpers.java
 * 2. Put it in your frc/robot/ folder
 * 3. In Limelight web interface, enable MegaTag2 in your AprilTag pipeline
 */
public class LimelightSubsystem extends SubsystemBase {

    // Limelight name (default is "limelight", change if you renamed yours)
    private static final String LIMELIGHT_NAME = "limelight-april";
    
    // NetworkTable for Limelight communication
    private final NetworkTable limelightTable;
    
    // NetworkTable entries for common Limelight values
    private final NetworkTableEntry tv;  // Whether we have a valid target (0 or 1)
    private final NetworkTableEntry tx;  // Horizontal offset (-29.8 to 29.8 degrees)
    private final NetworkTableEntry ty;  // Vertical offset (-24.85 to 24.85 degrees)
    private final NetworkTableEntry ta;  // Target area (0% to 100% of image)
    private final NetworkTableEntry tid; // AprilTag ID
    
    // Cache MegaTag2 data for consistent reads within a loop
    private double cachedDistanceMeters = 0;
    private boolean megatag2Available = false;
    
    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");
        
        // Set Limelight to AprilTag pipeline
        setPipeline(0);
    }
    
    // =========================================================================
    // BASIC LIMELIGHT DATA
    // =========================================================================
    
    public boolean hasValidTarget() {
        return tv.getDouble(0) == 1;
    }
    
    /** @return horizontal offset in DEGREES (positive = target is to the right) */
    public double getHorizontalOffset() {
        return tx.getDouble(0.0);
    }
    
    /** @return vertical offset in DEGREES */
    public double getVerticalOffset() {
        return ty.getDouble(0.0);
    }
    
    public double getTargetArea() {
        return ta.getDouble(0.0);
    }
    
    public int getAprilTagID() {
        return (int) tid.getInteger(-1);
    }
    
    public boolean isTargetID(int targetID) {
        return hasValidTarget() && getAprilTagID() == targetID;
    }
    
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    public void setLEDMode(boolean on) {
        limelightTable.getEntry("ledMode").setNumber(on ? 3 : 1);
    }
    
    // =========================================================================
    // DISTANCE METHODS - ALL RETURN METERS
    // =========================================================================
    
    /**
     * Get distance to target using MegaTag2 (camera-space Z)
     * @return distance in METERS, or -1 if no valid measurement
     */
    public double getDistanceMeters() {
        if (!megatag2Available) {
            return -1;
        }
        return cachedDistanceMeters;
    }
    
    /**
     * Get distance in INCHES (for SmartDashboard display only)
     * @return distance in inches, or -1 if no valid measurement
     */
    public double getDistanceInchesForDisplay() {
        if (!megatag2Available) {
            return -1;
        }
        return Units.metersToInches(cachedDistanceMeters);
    }
    
    /**
     * Check if MegaTag2 data is available this loop
     */
    public boolean hasMegaTag2Data() {
        return megatag2Available;
    }

    /**
     * Get the robot's estimated pose from MegaTag2
     * Useful for pose estimation fusion with odometry
     */
    public Pose2d getRobotPose_MegaTag2() {
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (result != null && result.tagCount > 0) {
            return result.pose;
        }
        return null;
    }
    
    /**
     * Get timestamp of MegaTag2 measurement (for pose estimation)
     */
    public double getMegaTag2Timestamp() {
        var result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        if (result != null && result.tagCount > 0) {
            return result.timestampSeconds;
        }
        return 0;
    }
    
    // =========================================================================
    // PERIODIC - Updates MegaTag2 cache
    // =========================================================================
    
    @Override
    public void periodic() {
        // Update MegaTag2 cache
        updateMegaTag2Cache();
        
        // Post data to SmartDashboard for debugging
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight/TX (deg)", getHorizontalOffset());
        SmartDashboard.putNumber("Limelight/TY (deg)", getVerticalOffset());
        SmartDashboard.putNumber("Limelight/Area", getTargetArea());
        SmartDashboard.putNumber("Limelight/AprilTag ID", getAprilTagID());
        SmartDashboard.putBoolean("Limelight/MegaTag2 Available", megatag2Available);
        
        if (megatag2Available) {
            // Display in INCHES for human readability
            SmartDashboard.putNumber("Limelight/Distance (in)", getDistanceInchesForDisplay());
            SmartDashboard.putNumber("Limelight/Distance (m)", cachedDistanceMeters);
        }
    }
    
    /**
     * Update MegaTag2 cache - called in periodic()
     * This ensures consistent data within a single loop iteration
     */
    private void updateMegaTag2Cache() {
        try {
            // Get target pose in camera space (gives us distance directly)
            double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
            
            // targetPose array: [x, y, z, roll, pitch, yaw]
            // targetPose[2] = Z distance (forward from camera) in meters
            if (targetPose != null && targetPose.length >= 3 && hasValidTarget()) {
                cachedDistanceMeters = targetPose[2];
                megatag2Available = true;
            } else {
                megatag2Available = false;
            }
        } catch (Exception e) {
            // LimelightHelpers might not be available or configured
            megatag2Available = false;
        }
    }

   
}