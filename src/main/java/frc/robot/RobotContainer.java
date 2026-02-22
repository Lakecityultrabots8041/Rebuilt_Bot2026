// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shoot.ShooterConstants;
import frc.robot.subsystems.shoot.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystems;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
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
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystems intakeSubsystem = new IntakeSubsystems();

    // Tracked commanded speeds for asymmetric slew (fast accel, controlled decel)
    private double currentDriveX = 0.0;
    private double currentDriveY = 0.0;

    // Auto-aim
    private boolean autoAimEnabled = false;
    private final PIDController autoAimPID = new PIDController(
        VisionConstants.AUTO_AIM_KP, VisionConstants.AUTO_AIM_KI, VisionConstants.AUTO_AIM_KD);
    private final SlewRateLimiter autoAimSlew = new SlewRateLimiter(VisionConstants.AUTO_AIM_SLEW_RATE);

    private final SendableChooser<Command> autoChooser;

    // Command factories — each call returns a new instance
    private Limelight_Move createHubAlign() {
        return new Limelight_Move(drivetrain, limelight,
            VisionConstants::getHubTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createTowerAlign() {
        return new Limelight_Move(drivetrain, limelight,
            VisionConstants::getTowerTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createOutpostAlign() {
        return new Limelight_Move(drivetrain, limelight,
            VisionConstants::getOutpostTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createAutoHubAlign() {
        System.out.println("Aligning to hub...");
        return new Limelight_Move(drivetrain, limelight, VisionConstants::getHubTags);
    }

    public RobotContainer() {
        limelight.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelight.setVisionMeasurementConsumer(
            (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
        autoAimPID.enableContinuousInput(-Math.PI, Math.PI);

        // PathPlanner named commands
        NamedCommands.registerCommand("Align Hub", createAutoHubAlign());

        // =====Shooter Name Commands=====
        NamedCommands.registerCommand("Rev Shooter", ShooterCommands.revUp(shooterSubsystem));
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot(shooterSubsystem));
        NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idle(shooterSubsystem));
        NamedCommands.registerCommand("Pass", ShooterCommands.passSequence(shooterSubsystem));
        NamedCommands.registerCommand("AlignAndShoot",
            Commands.sequence(createAutoHubAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));
        // =====Intake Commands=====
        NamedCommands.registerCommand("Intake", IntakeCommands.intake(intakeSubsystem));
        NamedCommands.registerCommand("Eject", IntakeCommands.eject(intakeSubsystem));
        NamedCommands.registerCommand("Idle Intake", IntakeCommands.idle(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Stow", IntakeCommands.pivotToStow(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Intake", IntakeCommands.pivotToIntake(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Travel", IntakeCommands.pivotToTravel(intakeSubsystem));
        NamedCommands.registerCommand("Start Intake", IntakeCommands.startingIntakeSequence(intakeSubsystem));
        NamedCommands.registerCommand("End Intake", IntakeCommands.endingIntakeSequence(intakeSubsystem));

        autoChooser = AutoBuilder.buildAutoChooser("Blue Mid Backup Auto");
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

                if (autoAimEnabled && limelight.isTrackingHubTag()) {
                    double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
                    double targetHeading = currentHeading - Math.toRadians(limelight.getHorizontalOffset());

                    double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
                    double clampedOutput = MathUtil.clamp(rawOutput,
                        -VisionConstants.AUTO_AIM_MAX_ROTATION_RATE,
                         VisionConstants.AUTO_AIM_MAX_ROTATION_RATE);
                    double smoothOutput = autoAimSlew.calculate(clampedOutput);

                    // Pre-spin shooter to distance-based velocity only when trigger is not held.
                    // When trigger is held, shooter is already firing — don't fight it.
                    double distance = limelight.getDistanceMeters();
                    if (distance > 0 && !controller.rightTrigger().getAsBoolean()) {
                        shooterSubsystem.setVariableVelocity(
                            ShooterConstants.getVelocityForDistance(distance));
                    }

                    return drive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(smoothOutput);
                } else {
                    autoAimSlew.reset(0);
                    autoAimPID.reset();
                    shooterSubsystem.clearVariableVelocity();

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
        controller.y().whileTrue(createTowerAlign());
        controller.x().whileTrue(createOutpostAlign());

        controller.rightTrigger().whileTrue(ShooterCommands.shoot(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));

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
        //controller.leftTrigger().whileTrue(ShooterCommands.ejectSequence(shooterSubsystem));
        controller.b().whileTrue(ShooterCommands.passSequence(shooterSubsystem));

        // ----- INTAKE ----
        // Left DPad to intake, right DPad to stop
        controller.povLeft().onTrue(IntakeCommands.intake(intakeSubsystem));
        controller.povRight().onTrue(IntakeCommands.idle(intakeSubsystem));

        // Pivot presets: DPad Up = stow, DPad Down = intake, A = travel (ramp safe)
        controller.povUp().onTrue(IntakeCommands.pivotToStow(intakeSubsystem));
        controller.povDown().onTrue(IntakeCommands.pivotToIntake(intakeSubsystem));
        controller.a().onTrue(IntakeCommands.pivotToTravel(intakeSubsystem));
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
        boolean visionLocked = autoAimEnabled && limelight.isTrackingHubTag();
        boolean intakeDown   = intakeSubsystem.getPivotState() == IntakeSubsystems.PivotState.INTAKE
                               && intakeSubsystem.isPivotAtTarget();

        SmartDashboard.putBoolean("Driver/Ready to Shoot", shooterReady && visionLocked);
        SmartDashboard.putBoolean("Driver/Shooter Ready",  shooterReady);
        SmartDashboard.putBoolean("Driver/Vision Locked",  visionLocked);
        SmartDashboard.putBoolean("Driver/Auto Aim On",    autoAimEnabled);
        SmartDashboard.putBoolean("Driver/Intake Down",    intakeDown);
    }

    public LimelightSubsystem getLimelight() { return limelight; }
    public ShooterSubsystem getShooter() { return shooterSubsystem; }
    public IntakeSubsystems getIntake() { return intakeSubsystem; }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}