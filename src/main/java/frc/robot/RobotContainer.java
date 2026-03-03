// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shoot.ShooterConstants;
import frc.robot.subsystems.shoot.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystems;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.commands.FollowTag_Demo;
import frc.robot.commands.Limelight_Move;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveConstants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

@SuppressWarnings("unused")
public class RobotContainer {

    private final CommandXboxController controller = new CommandXboxController(0);

    // Drivetrain
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Shooter camera ("limelight-april"), faces the shooter side. Used for alignment and auto-aim.
    private final LimelightSubsystem limelightShooter = new LimelightSubsystem("limelight-april", false);
    // Intake camera ("limelight-intake"), faces the intake side. Provides rear vision and pose fusion.
    private final LimelightSubsystem limelightIntake = new LimelightSubsystem("limelight-intake", true);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystems intakeSubsystem = new IntakeSubsystems();

    private final LEDSubsystem ledSubsystem;

    // Tracked commanded speeds for asymmetric slew (fast accel, controlled decel)
    private double currentDriveX = 0.0;
    private double currentDriveY = 0.0;

    // Auto-aim
    private boolean autoAimEnabled = false;
    private final PIDController autoAimPID = new PIDController(
        VisionConstants.AUTO_AIM_KP, VisionConstants.AUTO_AIM_KI, VisionConstants.AUTO_AIM_KD);
    private final SlewRateLimiter autoAimSlew = new SlewRateLimiter(VisionConstants.AUTO_AIM_SLEW_RATE);

    private final SendableChooser<Command> autoChooser;

    // Command factories -- each call returns a new instance.
    // All alignment currently uses the shooter camera.
    private Limelight_Move createHubAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getHubTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createTowerAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getTowerTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createOutpostAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getOutpostTags, () -> -controller.getLeftX());
    }

    // Auto (no driver input) factories for PathPlanner named commands
    private Limelight_Move createAutoHubAlign() {
        return new Limelight_Move(drivetrain, limelightShooter, VisionConstants::getHubTags);
    }

    private Limelight_Move createAutoOutpostAlign() {
        return new Limelight_Move(drivetrain, limelightShooter, VisionConstants::getOutpostTags);
    }

    private Limelight_Move createAutoTowerAlign() {
        return new Limelight_Move(drivetrain, limelightShooter, VisionConstants::getTowerTags);
    }

    private Limelight_Move createAutoTrenchAlign() {
        return new Limelight_Move(drivetrain, limelightShooter, VisionConstants::getTrenchTags);
    }

    public RobotContainer() {
        // Both cameras fuse poses into the drivetrain Kalman filter for better localization
        limelightShooter.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelightShooter.setVisionMeasurementConsumer(
            (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));

        limelightIntake.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelightIntake.setVisionMeasurementConsumer(
            (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
        autoAimPID.enableContinuousInput(-Math.PI, Math.PI);

        // PathPlanner named commands - vision alignment
        NamedCommands.registerCommand("Align Hub", createAutoHubAlign());
        NamedCommands.registerCommand("Align Outpost", createAutoOutpostAlign());
        NamedCommands.registerCommand("Align Tower", createAutoTowerAlign());
        NamedCommands.registerCommand("Align Trench", createAutoTrenchAlign());

        // =====Shooter Name Commands=====
        NamedCommands.registerCommand("Rev Flywheel", ShooterCommands.revUpFlywheel(shooterSubsystem));
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot(shooterSubsystem));
        NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idle(shooterSubsystem));
        NamedCommands.registerCommand("Pass", ShooterCommands.passSequence(shooterSubsystem));
        NamedCommands.registerCommand("Quick Shoot", ShooterCommands.quickShoot(shooterSubsystem));
        NamedCommands.registerCommand("AlignHubAndShoot",
            Commands.sequence(createAutoHubAlign().andThen(ShooterCommands.quickShoot(shooterSubsystem))));
        NamedCommands.registerCommand("AlignOutpostAndShoot",
            Commands.sequence(createAutoOutpostAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));
        NamedCommands.registerCommand("AlignTowerAndShoot",
            Commands.sequence(createAutoTowerAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));
        // =====Intake Commands=====
        NamedCommands.registerCommand("Intake", IntakeCommands.intake(intakeSubsystem));
        NamedCommands.registerCommand("Eject", IntakeCommands.eject(intakeSubsystem));
        NamedCommands.registerCommand("Idle Intake", IntakeCommands.idle(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Stow", IntakeCommands.pivotToStow(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Intake", IntakeCommands.pivotToIntake(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Travel", IntakeCommands.pivotToTravel(intakeSubsystem));
        NamedCommands.registerCommand("Start Intake", IntakeCommands.startingIntakeSequence(intakeSubsystem));
        NamedCommands.registerCommand("End Intake", IntakeCommands.endingIntakeSequence(intakeSubsystem));

        // LEDs react to subsystem states automatically, no commands needed
        ledSubsystem = new LEDSubsystem(
            () -> shooterSubsystem.isFlywheelReady()
                  && shooterSubsystem.getFeedState() == ShooterSubsystem.FeedState.FEEDING,
            () -> autoAimEnabled && limelightShooter.isTrackingHubTag(),
            () -> autoAimEnabled,
            () -> intakeSubsystem.getState() == IntakeSubsystems.IntakeState.INTAKING
        );

        autoChooser = AutoBuilder.buildAutoChooser("SimplePath Auton");
        SmartDashboard.putData("Auton Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Default drive — field-centric with auto-aim overlay
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                currentDriveX = applyDriveSlew(currentDriveX, -controller.getLeftY() * MaxSpeed);
                currentDriveY = applyDriveSlew(currentDriveY, -controller.getLeftX() * MaxSpeed);
                double velocityX = currentDriveX;
                double velocityY = currentDriveY;

                if (autoAimEnabled && limelightShooter.isTrackingHubTag()) {
                    double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
                    double targetHeading = currentHeading - Math.toRadians(limelightShooter.getHorizontalOffset());

                    double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
                    double clampedOutput = MathUtil.clamp(rawOutput,
                        -VisionConstants.AUTO_AIM_MAX_ROTATION_RATE,
                         VisionConstants.AUTO_AIM_MAX_ROTATION_RATE);
                    double smoothOutput = autoAimSlew.calculate(clampedOutput);

                    // Pre-spin shooter to distance-based velocity only when trigger is not held.
                    // When trigger is held, shooter is already firing — don't fight it.
                    double distance = limelightShooter.getDistanceMeters();
                    if (distance > 0 && !controller.rightTrigger().getAsBoolean()) {
                        shooterSubsystem.setAutoAimSpeed(
                            ShooterConstants.getSpeedForDistance(distance));
                    }

                    return drive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(smoothOutput);
                } else {
                    autoAimSlew.reset(0);
                    autoAimPID.reset();
                    shooterSubsystem.clearAutoAimSpeed();

                    return drive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(-controller.getRightX() * MaxAngularRate);
                }
            })
        );

        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Auto-aim toggle
        controller.leftBumper().onTrue(Commands.runOnce(() -> {
            autoAimEnabled = !autoAimEnabled;
            SmartDashboard.putBoolean("AutoAim/Enabled", autoAimEnabled);
        }));

        // Vision alignment: START=hub, Y=tower, X=outpost
        controller.start().whileTrue(createHubAlign());
        controller.back().whileTrue(createTowerAlign());
        controller.y().whileTrue(createOutpostAlign());

        //TODO If wrong change back to normal
        if (controller.povRight().getAsBoolean() == true) {
            controller.rightTrigger().whileTrue(ShooterCommands.testShot(shooterSubsystem))
                .onFalse(ShooterCommands.idle(shooterSubsystem));
        } else {
        controller.rightTrigger().whileTrue(ShooterCommands.shoot(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));
        }
        

        // Right Trigger plan if we want auto align on shoot, 
        // Align to hub in parallel with spinning up, then confirm on target before firing                                                        
       /*  controller.rightTrigger().whileTrue(                                                                                                      
                Commands.sequence(                                                                                                                    
                Commands.parallel(                                                                                                                
                        createHubAlign(),                                                                                                             
                        shooterSubsystem.shoot()                                                                                                      
                        ),                                                                                                                                
                        shooterSubsystem.waitUntilReady(),                                                                                                
                        Commands.waitSeconds(0.5),                                                                                                        
                        shooterSubsystem.idle()                                                                                                           
        ) 
    */                                                                                                                                    
        controller.leftTrigger().whileTrue(ShooterCommands.eject(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));
        controller.b().whileTrue(ShooterCommands.pass(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));

        // ----- INTAKE ----
        // x to intake, a to stop
        controller.x().whileTrue(IntakeCommands.intake(intakeSubsystem))
            .onFalse(IntakeCommands.idle(intakeSubsystem));
        
        
        //controller.a().onTrue(IntakeCommands.idle(intakeSubsystem));

        // Pivot presets: DPad Up = stow, DPad Down = intake, A = travel (ramp safe)
        controller.povUp().onTrue(IntakeCommands.pivotToStow(intakeSubsystem));
        controller.povDown().onTrue(IntakeCommands.pivotToIntake(intakeSubsystem));
        controller.a().onTrue(IntakeCommands.pivotToTravel(intakeSubsystem));

        // Demo: hold DPad Left to follow any visible AprilTag at safe distance
        controller.povLeft().whileTrue(new FollowTag_Demo(drivetrain, limelightShooter));
    }

    /**
     * Asymmetric slew rate limiter.
     * Accelerating (|requested| > |current|) uses MAX_TELEOP_ACCEL.
     * Decelerating or reversing  uses MAX_TELEOP_DECEL.
     * Tune both values in DriveConstants.java.
     */
    private double applyDriveSlew(double current, double requested) {
        double rateLimit = (Math.abs(requested) > Math.abs(current))
            ? DriveConstants.MAX_TELEOP_ACCEL
            : DriveConstants.MAX_TELEOP_DECEL;
        double maxDelta = rateLimit * 0.02; // 20 ms loop period
        return current + MathUtil.clamp(requested - current, -maxDelta, maxDelta);
    }

    public void updateDriverDashboard() {
        boolean shooterReady = shooterSubsystem.atTargetVelocity() && shooterSubsystem.atFlywheelTargetVelocity();
        boolean visionLocked = autoAimEnabled && limelightShooter.isTrackingHubTag();
        boolean intakeDown   = intakeSubsystem.getPivotState() == IntakeSubsystems.PivotState.INTAKE
                               && intakeSubsystem.isPivotAtTarget();

        SmartDashboard.putBoolean("Driver/Ready to Shoot", shooterReady && visionLocked);
        SmartDashboard.putBoolean("Driver/Shooter Ready",  shooterReady);
        SmartDashboard.putBoolean("Driver/Vision Locked",  visionLocked);
        SmartDashboard.putBoolean("Driver/Auto Aim On",    autoAimEnabled);
        SmartDashboard.putBoolean("Driver/Intake Down",    intakeDown);
    }

    public LimelightSubsystem getLimelightShooter() { return limelightShooter; }
    public LimelightSubsystem getLimelightIntake() { return limelightIntake; }
    public ShooterSubsystem getShooter() { return shooterSubsystem; }
    public IntakeSubsystems getIntake() { return intakeSubsystem; }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}