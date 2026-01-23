package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
    //private final double maxSpeedMetersPerSec;
    //private final double maxRotationsPerSec;
    
    // Timer for timeout
    private final Timer timer;

    //TODO If this doesn't fix the jerky issue remove all instances of aligning
    //Experimental to continue alignment loop below
    private double aligning = 0; // 0=Not aligning, 1=Currently aligning
    
    // Track if we've been aligned consistently
    private int alignedCount = 0;
    private static final int REQUIRED_ALIGNED_LOOPS = 25; // Must be aligned for 25 loops (0.5 sec)

    //How close we are trying to get with the current tag
    private double targetDistance = Constants.defaultAprilTagDistance;
    
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

        //How far the tag we see is
        if (limelight.hasValidTarget()) {
            int tagID = limelight.getAprilTagID();
            targetDistance = Constants.getAprilTagDistance(tagID);
            System.out.println("Aligning to tag" + tagID +
                                "at" + Units.metersToInches(targetDistance) + "Inches");
        }
    }
    //FIXME This is what seems to keep getting interrupted and making the robot alignment jerky
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

        int tagID = limelight.getAprilTagID();
        targetDistance = Constants.getAprilTagDistance(tagID);
        
        SmartDashboard.putString("Vision/Status", "Tracking tag" + tagID);
        
        //How far is the tag (Inches), and how far off center is it? hopefully replace PIDs
        double degreesOffCenter = limelight.getHorizontalOffset();
        double CurrentDistance = limelight.getDistanceInches_MegaTag2();
        if(CurrentDistance < 0) {
            //No distance data
            drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Vision/Status", "No distance data found");
        }
        // Get current error values from Limelight
        double horizontalError = limelight.getHorizontalOffset(); // tx in degrees
        //if (limelight.getDistanceInches_MegaTag2() >= 0) {
        double currentDistance = limelight.getDistanceInches_MegaTag2(
        
        );
        //TODO find a way to hopefully keep track of the last time our bot saw a tag and use that(?)
       // } else {
            //double currentDistance = cachedDistanceMeters()
        //};
        //How close/far are we?(Inch)
        double distanceError = targetDistance - CurrentDistance;

        //How far left/right are we?(Inch)
        double InchesOffCenter = CurrentDistance * Math.tan(Math.toRadians(degreesOffCenter));

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

        //Figure out how fast we want to go Clockwise, Forward, and Around(right)
        double RotationSpeed = degreesOffCenter * Constants.rotationGain;
        double ForwardSpeed = distanceError * Constants.forwardGain;
        double StrafeSpeed = InchesOffCenter * Constants.strafeGain;

        //TODO This is where I start replacing things, if something breaks, revert all changes below this (generally just capitalizing)
        
        // Clamp speeds to maximum values
        RotationSpeed = clamp(RotationSpeed, -Constants.MAX_ROTATION_SPEED, Constants.MAX_ROTATION_SPEED);
        ForwardSpeed = clamp(ForwardSpeed, -Constants.MAX_DISTANCE_SPEED, Constants.MAX_DISTANCE_SPEED);
        StrafeSpeed = clamp(StrafeSpeed, -Constants.MAX_STRAFE_SPEED, Constants.MAX_STRAFE_SPEED);
        
        // Convert to actual velocities
        double RotationVelocity = -RotationSpeed * maxAngularRate; 
        double ForwardVelocity = ForwardSpeed * 0;
        double StrafeVelocity = StrafeSpeed * maxSpeed;
        
        // Create chassis speeds (robot-relative)
        // X = forward/backward, Y = left/right strafe, Omega = rotation
        ChassisSpeeds speeds = new ChassisSpeeds(ForwardVelocity, StrafeVelocity,  RotationVelocity);
        
        // Send speeds to drivetrain
        drivetrain.driveRobotRelative(speeds);
        
        // Debug output to SmartDashboard
        SmartDashboard.putNumber("Vision/Horizontal Error", horizontalError);
        SmartDashboard.putNumber("Vision/Horizontal Offset (in)", horizontalOffsetInches);
        SmartDashboard.putNumber("Vision/Distance", CurrentDistance);
        SmartDashboard.putNumber("Vision/Rotation Speed", RotationSpeed);
        SmartDashboard.putNumber("Vision/Forward Speed", ForwardSpeed);
        SmartDashboard.putNumber("Vision/Strafe Speed", StrafeSpeed);
        SmartDashboard.putBoolean("Vision/At Rotation", rotationController.atSetpoint());
        SmartDashboard.putBoolean("Vision/At Distance", distanceController.atSetpoint());
        SmartDashboard.putBoolean("Vision/At Strafe", strafeController.atSetpoint());
        /* 
        // Check if we're aligned on ALL axes
        if (rotationController.atSetpoint() && distanceController.atSetpoint() && strafeController.atSetpoint()) {
            alignedCount++;
            SmartDashboard.putNumber("Vision/Aligned Count", alignedCount);
            aligning = 0;
        } else {
            alignedCount = 0;
            aligning = 1;
        }
            */

            //Close Enough?
            boolean rotationGood = Math.abs(degreesOffCenter) < Constants.ALIGNMENT_TOLERANCE_DEGREES;
            boolean distanceGood = Math.abs(distanceError) < Constants.DISTANCE_TOLERANCE_INCHES;

            SmartDashboard.putBoolean("Vision/Rotation Aligned", rotationGood);
            SmartDashboard.putBoolean("Vision/Distance Aligned", distanceGood);

            //How many Loops have we been aligned for?
            if (rotationGood && distanceGood) {
                alignedCount++;
                SmartDashboard.putNumber("Vision/Loops Aligned", alignedCount);
            } else {
                alignedCount = 0;
            }
            
    }
    //FIXME This is what keeps interrupting the above alignment attempt, we need some way to stop the above code from being interrupted(maybe adding a target pose & bot's guessed pose?)
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
