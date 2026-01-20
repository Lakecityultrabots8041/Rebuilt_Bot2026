// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

      
    // ===== APRILTAG TARGETING =====
    /** The AprilTag ID we want to align with for scoring */
    public static final int TARGET_APRILTAG_ID = 15;
    
    // ===== CAMERA MOUNTING =====
    /** Height of the Limelight lens from the floor in inches */
    public static final double CAMERA_HEIGHT_INCHES = 16; // ADJUST THIS to your robot
    
    /** Angle of the camera mounting in degrees (0 = horizontal, positive = angled up) */
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 0; // ADJUST THIS to your robot
    
    /** Height of the AprilTag center from the floor in inches */
    public static final double APRILTAG_HEIGHT_INCHES = 57.0; // Standard FRC field AprilTag height
    
    // ===== ALIGNMENT GOALS =====
    /** Target distance from AprilTag in inches (how close we want to get) */
    public static final double TARGET_DISTANCE_INCHES = 60.0; // 5 feet - safe for testing
    
    /** How much horizontal offset (in degrees) is acceptable before we're "aligned" */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;
    
    /** How much distance error (in inches) is acceptable before we're "at distance" */
    public static final double DISTANCE_TOLERANCE_INCHES = 5.0;

   
    
    // ===== PID CONSTANTS FOR ROTATION (turning to face tag) =====
    public static final double ROTATION_KP = 0.02;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.005;
    public static final double MAX_ROTATION_SPEED = 0.2; // 20% of max speed
    
    // ===== PID CONSTANTS FOR FORWARD/BACKWARD (distance control) =====
    public static final double DISTANCE_KP = 0.01;
    public static final double DISTANCE_KI = 0.0;
    public static final double DISTANCE_KD = 0.003;
    public static final double MAX_DISTANCE_SPEED = 0.15; // 15% of max speed
    
    // ===== PID CONSTANTS FOR STRAFE (left/right centering) =====
    // *** THESE WERE MISSING - Added to fix compile errors ***
    /** 
     * Proportional gain for strafe - controls how aggressively robot slides left/right
     * Higher = faster correction, but can overshoot
     */
    public static final double STRAFE_KP = 0.02;
    
    /** Integral gain for strafe - usually keep at 0 */
    public static final double STRAFE_KI = 0.0;
    
    /** Derivative gain for strafe - helps prevent oscillation */
    public static final double STRAFE_KD = 0.003;
    
    /** Maximum strafe speed as fraction of max speed (0.0 to 1.0) */
    public static final double MAX_STRAFE_SPEED = 0.15; // 15% of max speed
    
    /** How much strafe offset (in inches) is acceptable before we're "centered" */
    public static final double STRAFE_TOLERANCE_INCHES = 3.0;
    
    // ===== SAFETY =====
    /** Minimum target area to consider valid (prevents false positives from far away) */
    public static final double MIN_TARGET_AREA = 0.1;
    
    /** Maximum time to run alignment command before giving up (seconds) */
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 8.0;
}
