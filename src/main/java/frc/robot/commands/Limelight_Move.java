package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import static edu.wpi.first.units.Units.*;

/**
 * Vision alignment command that locks rotation and distance to an AprilTag
 * while allowing the driver to strafe (arc) around the target.
 * 
 * Control architecture:
 * - Rotation (omega): AUTO - keeps robot facing the tag
 * - Forward/Back (vx): AUTO - maintains target distance
 * - Strafe (vy): DRIVER - allows arcing around the target
 * 
 * ALL INTERNAL CALCULATIONS USE METERS.
 */
public class Limelight_Move extends Command {
        
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final int targetID;
    private final DoubleSupplier driverStrafeSupplier;
    
    // Max speeds from TunerConstants (in m/s and rad/s)
    private final double maxSpeedMps;
    private final double maxAngularRateRps;
    
    // Timer for timeout
    private final Timer timer;
    
    // Track alignment consistency
    private int alignedCount = 0;

    // Target distance for current tag (in METERS)
    private double targetDistanceMeters;
    
    /**
     * Creates a new alignment command for the default AprilTag (15)
     * with no driver strafe input (pure auto-alignment)
     */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        this(drivetrain, limelight, Constants.TARGET_APRILTAG_ID, () -> 0.0);
    }
    
    /**
     * Creates a new alignment command with driver strafe input for arcing
     * 
     * @param drivetrain the swerve drivetrain
     * @param limelight the Limelight subsystem
     * @param driverStrafeSupplier supplier for driver's strafe input (-1 to 1, typically leftX)
     */


    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, 
                          DoubleSupplier driverStrafeSupplier) {
        this(drivetrain, limelight, Constants.TARGET_APRILTAG_ID, driverStrafeSupplier);
    }
    
    /**
     * Creates a new alignment command for a specific AprilTag with driver strafe
     * 
     * @param drivetrain the swerve drivetrain
     * @param limelight the Limelight subsystem
     * @param targetID the AprilTag ID to align with
     * @param driverStrafeSupplier supplier for driver's strafe input (-1 to 1)
     */



     
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, 
                          int targetID, DoubleSupplier driverStrafeSupplier) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.targetID = targetID;
        this.driverStrafeSupplier = driverStrafeSupplier;
        
        // Get max speeds from TunerConstants
        maxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRateRps = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        
        timer = new Timer();
        targetDistanceMeters = Constants.defaultAprilTagDistance;
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== Vision Alignment Started ===");
        System.out.println("Looking for AprilTag ID: " + targetID);
        
        // Turn on Limelight LEDs
        limelight.setLEDMode(true);
        
        // Reset aligned counter
        alignedCount = 0;
        
        // Start timeout timer
        timer.restart();

        // Get target distance for the tag we're tracking
        if (limelight.hasValidTarget()) {
            int tagID = limelight.getAprilTagID();
            targetDistanceMeters = Constants.getAprilTagDistance(tagID);
            System.out.println("Aligning to tag " + tagID + " at " + 
                               Units.metersToInches(targetDistanceMeters) + " inches");
        }
    }
    
    @Override
    public void execute() {
        // Check if we can see the target
        if (!limelight.isTargetID(targetID)) {
            // No target - stop the robot
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "No Target ID " + targetID);
            alignedCount = 0;
            return;
        }

        // Update target distance in case we switched tags
        int tagID = limelight.getAprilTagID();
        targetDistanceMeters = Constants.getAprilTagDistance(tagID);
        
        SmartDashboard.putString("Vision/Status", "Tracking tag " + tagID);
        
        // Get current measurements
        double currentDistanceMeters = limelight.getDistanceMeters();
        
        // Check for valid distance data
        if (currentDistanceMeters < 0) {
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "No distance data");
            alignedCount = 0;
            return;
        }
        
        // Get horizontal offset in degrees (positive = target is to the RIGHT)
        double horizontalErrorDeg = limelight.getHorizontalOffset();
        
        // Calculate distance error (positive = too far away)
        double distanceErrorMeters = currentDistanceMeters - targetDistanceMeters;
        
        // =====================================================================
        // ROTATION CONTROL (AUTO)
        // Positive TX means target is RIGHT, so we need NEGATIVE omega (clockwise)
        // to turn right and face the target
        // =====================================================================
        double rotationOutput = -horizontalErrorDeg * Constants.ROTATION_GAIN;
        rotationOutput = MathUtil.clamp(rotationOutput, -Constants.MAX_ROTATION_SPEED, Constants.MAX_ROTATION_SPEED);
        double rotationVelocityRps = rotationOutput * maxAngularRateRps;
        
        // =====================================================================
        // FORWARD CONTROL (AUTO)
        // Positive error (too far) → positive velocity (drive forward)
        // =====================================================================
        double forwardOutput = distanceErrorMeters * Constants.FORWARD_GAIN;
        forwardOutput = MathUtil.clamp(forwardOutput, -Constants.MAX_FORWARD_SPEED, Constants.MAX_FORWARD_SPEED);
        double forwardVelocityMps = forwardOutput * maxSpeedMps;
        
        // =====================================================================
        // STRAFE CONTROL (DRIVER)
        // This allows the driver to arc around the target while maintaining
        // rotation lock and distance. Creates smooth orbiting motion.
        // =====================================================================
        double driverStrafe = driverStrafeSupplier.getAsDouble();
        // Apply deadband
        driverStrafe = Math.abs(driverStrafe) < 0.1 ? 0.0 : driverStrafe;
        // Scale for safety during alignment
        double strafeVelocityMps = driverStrafe * maxSpeedMps * Constants.MAX_DRIVER_STRAFE_SCALE;
        
        // =====================================================================
        // SEND TO DRIVETRAIN
        // Robot-relative: X = forward, Y = left, Omega = CCW positive
        // =====================================================================
        ChassisSpeeds speeds = new ChassisSpeeds(
            forwardVelocityMps,   // X: forward/backward
            strafeVelocityMps,    // Y: left/right (driver controlled)
            rotationVelocityRps   // Omega: rotation
        );
        
        drivetrain.driveRobotRelative(speeds);
        
        // =====================================================================
        // DEBUG OUTPUT
        // =====================================================================
        SmartDashboard.putNumber("Vision/TX Error (deg)", horizontalErrorDeg);
        SmartDashboard.putNumber("Vision/Distance (in)", Units.metersToInches(currentDistanceMeters));
        SmartDashboard.putNumber("Vision/Target Distance (in)", Units.metersToInches(targetDistanceMeters));
        SmartDashboard.putNumber("Vision/Distance Error (in)", Units.metersToInches(distanceErrorMeters));
        SmartDashboard.putNumber("Vision/Rotation Output", rotationOutput);
        SmartDashboard.putNumber("Vision/Forward Output", forwardOutput);
        SmartDashboard.putNumber("Vision/Driver Strafe", driverStrafe);
        
        // =====================================================================
        // ALIGNMENT CHECK
        // =====================================================================
        boolean rotationGood = Math.abs(horizontalErrorDeg) < Constants.ALIGNMENT_TOLERANCE_DEGREES;
        boolean distanceGood = Math.abs(distanceErrorMeters) < Constants.DISTANCE_TOLERANCE_METERS;
        
        SmartDashboard.putBoolean("Vision/Rotation Aligned", rotationGood);
        SmartDashboard.putBoolean("Vision/Distance Aligned", distanceGood);
        
        // Count consecutive aligned loops
        if (rotationGood && distanceGood) {
            alignedCount++;
            SmartDashboard.putNumber("Vision/Aligned Loops", alignedCount);
        } else {
            alignedCount = 0;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        
        // Turn off Limelight LEDs to save power
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
        // Finish if we've been aligned consistently
        if (alignedCount >= Constants.ALIGNED_LOOPS_REQUIRED) {
            System.out.println("Vision alignment achieved!");
            return true;
        }
        
        // Also finish if we timeout
        if (timer.hasElapsed(Constants.ALIGNMENT_TIMEOUT_SECONDS)) {
            System.out.println("Vision alignment timed out!");
            return true;
        }
        
        return false;
    }

}