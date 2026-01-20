package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import static edu.wpi.first.units.Units.*;


public class Limelight_Move extends Command {
        
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private final int targetID;
    
    // PID controllers for alignment
    private final PIDController rotationController;
    private final PIDController distanceController;
    private final PIDController strafeController; // NEW: For left/right centering
    
    // Speeds from TunerConstants
    private final double maxSpeed;
    private final double maxAngularRate;
    
    // Timer for timeout
    private final Timer timer;
    
    // Track if we've been aligned consistently
    private int alignedCount = 0;
    private static final int REQUIRED_ALIGNED_LOOPS = 25; // Must be aligned for 25 loops (0.5 sec)
    
    /**
     * Creates a new alignment command for AprilTag 15
     * @param drivetrain the swerve drivetrain
     * @param limelight the Limelight subsystem
     */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        this(drivetrain, limelight, Constants.TARGET_APRILTAG_ID);
    }
    
    /**
     * Creates a new alignment command for a specific AprilTag
     * @param drivetrain the swerve drivetrain
     * @param limelight the Limelight subsystem
     * @param targetID the AprilTag ID to align with
     */
    public Limelight_Move(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight, int targetID) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.targetID = targetID;
        
        // Create PID controllers
        rotationController = new PIDController(
            Constants.ROTATION_KP,
            Constants.ROTATION_KI,
            Constants.ROTATION_KD
        );
        
        distanceController = new PIDController(
            Constants.DISTANCE_KP,
            Constants.DISTANCE_KI,
            Constants.DISTANCE_KD
        );
        
        strafeController = new PIDController(
            Constants.STRAFE_KP,
            Constants.STRAFE_KI,
            Constants.STRAFE_KD
        );
        
        // Configure rotation PID
        rotationController.setSetpoint(0); // We want tx = 0 (centered)
        rotationController.setTolerance(Constants.ALIGNMENT_TOLERANCE_DEGREES);
        
        // Configure distance PID
        distanceController.setSetpoint(Constants.TARGET_DISTANCE_INCHES);
        distanceController.setTolerance(Constants.DISTANCE_TOLERANCE_INCHES);
        
        // Configure strafe PID - for left/right centering
        strafeController.setSetpoint(0); // We want to be centered (no offset)
        strafeController.setTolerance(Constants.STRAFE_TOLERANCE_INCHES);
        
        // Get max speeds from TunerConstants
        maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        
        timer = new Timer();
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== Vision Alignment Started ===");
        System.out.println("Looking for AprilTag ID: " + targetID);
        
        // Reset PID controllers
        rotationController.reset();
        distanceController.reset();
        strafeController.reset();
        
        // Turn on Limelight LEDs
        limelight.setLEDMode(true);
        
        // Reset aligned counter
        alignedCount = 0;
        
        // Start timeout timer
        timer.restart();
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
        
        SmartDashboard.putString("Vision/Status", "Target Acquired!");
        
        // Get current error values from Limelight
        double horizontalError = limelight.getHorizontalOffset(); // tx in degrees
        double currentDistance = limelight.getDistanceInches_MegaTag2(
            
        );
        
        // Calculate horizontal offset in inches for strafe
        // Use simple trig: offset = distance * tan(tx)
        double horizontalOffsetInches = currentDistance * Math.tan(Math.toRadians(horizontalError));
        
        // Calculate PID outputs
        // Rotation: positive tx means target is RIGHT, so we turn RIGHT (positive rotation)
        double rotationSpeed = rotationController.calculate(horizontalError);
        
        // Distance: positive when too far, drives forward (positive X)
        double forwardSpeed = distanceController.calculate(currentDistance);
        
        // Strafe: positive offset means we're to the LEFT of target, so strafe RIGHT (positive Y)
        double strafeSpeed = strafeController.calculate(horizontalOffsetInches);
        
        // Clamp speeds to maximum values
        rotationSpeed = clamp(rotationSpeed, -Constants.MAX_ROTATION_SPEED, Constants.MAX_ROTATION_SPEED);
        forwardSpeed = clamp(forwardSpeed, -Constants.MAX_DISTANCE_SPEED, Constants.MAX_DISTANCE_SPEED);
        strafeSpeed = clamp(strafeSpeed, -Constants.MAX_STRAFE_SPEED, Constants.MAX_STRAFE_SPEED);
        
        // Convert to actual velocities
        double rotationVelocity = rotationSpeed * maxAngularRate;
        double forwardVelocity = forwardSpeed * maxSpeed;
        double strafeVelocity = strafeSpeed * maxSpeed;
        
        // Create chassis speeds (robot-relative)
        // X = forward/backward, Y = left/right strafe, Omega = rotation
        ChassisSpeeds speeds = new ChassisSpeeds(forwardVelocity, strafeVelocity, rotationVelocity);
        
        // Send speeds to drivetrain
        drivetrain.driveRobotRelative(speeds);
        
        // Debug output to SmartDashboard
        SmartDashboard.putNumber("Vision/Horizontal Error", horizontalError);
        SmartDashboard.putNumber("Vision/Horizontal Offset (in)", horizontalOffsetInches);
        SmartDashboard.putNumber("Vision/Distance", currentDistance);
        SmartDashboard.putNumber("Vision/Rotation Speed", rotationSpeed);
        SmartDashboard.putNumber("Vision/Forward Speed", forwardSpeed);
        SmartDashboard.putNumber("Vision/Strafe Speed", strafeSpeed);
        SmartDashboard.putBoolean("Vision/At Rotation", rotationController.atSetpoint());
        SmartDashboard.putBoolean("Vision/At Distance", distanceController.atSetpoint());
        SmartDashboard.putBoolean("Vision/At Strafe", strafeController.atSetpoint());
        
        // Check if we're aligned on ALL axes
        if (rotationController.atSetpoint() && distanceController.atSetpoint() && strafeController.atSetpoint()) {
            alignedCount++;
            SmartDashboard.putNumber("Vision/Aligned Count", alignedCount);
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
        if (alignedCount >= REQUIRED_ALIGNED_LOOPS) {
            return true;
        }
        
        // Also finish if we timeout
        if (timer.hasElapsed(Constants.ALIGNMENT_TIMEOUT_SECONDS)) {
            System.out.println("Vision alignment timed out!");
            return true;
        }
        
        return false;
    }
    
    /**
     * Helper method to clamp a value between min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
