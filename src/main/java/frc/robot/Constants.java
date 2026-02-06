// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * <p>ALL DISTANCES IN THIS FILE ARE IN METERS unless explicitly noted otherwise.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    // ===== APRILTAG TARGET DISTANCES (METERS) =====
    // These define how far the robot should stop from each AprilTag
    public static final double[] aprilTagDistance = new double[18];
    static {
        // Default distance for unknown tags
        for (int i = 0; i < aprilTagDistance.length; i++) {
            aprilTagDistance[i] = 2.0; // 2 meters default
        }

        // Specific tag distances (ALL IN METERS)
        aprilTagDistance[9] = Units.inchesToMeters(97);  // 97 inches = 2.46 meters (main scoring target)
        aprilTagDistance[7] = Units.inchesToMeters(48);   // 48 inches = 1.22 meters
        aprilTagDistance[8] = Units.inchesToMeters(72);   // 72 inches = 1.83 meters
    }

    // Default distance if unknown tag
    public static final double defaultAprilTagDistance = 2.0; // meters

    // ===== APRILTAG TARGETING =====
    /** The AprilTag ID we want to align with for scoring */
    public static final int TARGET_APRILTAG_ID = 9;

    // ===== CAMERA MOUNTING (kept in inches for Limelight web UI reference) =====
    /** Height of the Limelight lens from the floor in inches */
    public static final double CAMERA_HEIGHT_INCHES = 12;
    
    /** Angle of the camera mounting in degrees (0 = horizontal, positive = angled up) */
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 45;
    
    /** Height of the AprilTag center from the floor in inches */
    public static final double APRILTAG_HEIGHT_INCHES = 57.0;

    // ===== ALIGNMENT TOLERANCES =====
    /** How much horizontal offset (in degrees) is acceptable before we're "aligned" */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    
    /** How much distance error (in METERS) is acceptable before we're "at distance" */
    public static final double DISTANCE_TOLERANCE_METERS = Units.inchesToMeters(3.0); // 3 inches = 0.076m

    // ===== PROPORTIONAL GAINS =====
    // Speed = error × gain
    // These convert error directly to speed output (0.0 to 1.0 range)
    
    /** Rotation gain: degrees of error → rotation speed */
    public static final double ROTATION_GAIN = 0.03;  // Slightly reduced for stability
    
    /** Forward gain: meters of error → forward speed */
    public static final double FORWARD_GAIN = 0.8;    // Increased because error is now in meters (small numbers)
    
    // ===== SPEED LIMITS (as fraction of max speed, 0.0 to 1.0) =====
    public static final double MAX_ROTATION_SPEED = 0.25;
    public static final double MAX_FORWARD_SPEED = 0.30;
    public static final double MAX_DRIVER_STRAFE_SCALE = 0.5;  // Driver strafe input scaled to 50% during alignment

    // ===== SAFETY =====
    /** Minimum target area to consider valid (prevents false positives from far away) */
    public static final double MIN_TARGET_AREA = 0.1;
    
    /** Maximum time to run alignment command before giving up (seconds) */
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 8.0;

    /** Number of consecutive aligned loops required before command finishes (1 loop ≈ 20ms) */
    public static final int ALIGNED_LOOPS_REQUIRED = 25; // ~0.5 seconds

// ===== SHOOTER =====
 public static class ShooterCostants {
    private ShooterCostants() {}
   
    public static final int SHOOTER_MOTOR = 2;
}


    /**
     * Get the target distance for a specific AprilTag ID
     * @param tagID the AprilTag ID
     * @return distance in METERS
     */
    public static double getAprilTagDistance(int tagID) {
        if (tagID >= 0 && tagID < aprilTagDistance.length) {
            return aprilTagDistance[tagID];
        }
        return defaultAprilTagDistance;
    }
}