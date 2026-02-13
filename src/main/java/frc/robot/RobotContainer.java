// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.climb.ClimberSubsystem;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shoot.ShooterConstants;
import frc.robot.subsystems.shoot.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.commands.Limelight_Move;
import frc.robot.commands.ShooterCommands;
import frc.robot.generated.TunerConstants;

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
        return new Limelight_Move(drivetrain, limelight, VisionConstants::getHubTags);
    }

    public RobotContainer() {
        limelight.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelight.setVisionMeasurementConsumer(
            (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
        autoAimPID.enableContinuousInput(-Math.PI, Math.PI);

        // PathPlanner named commands
        NamedCommands.registerCommand("Align Hub", createAutoHubAlign());
        NamedCommands.registerCommand("Rev Shooter", ShooterCommands.revUp(shooterSubsystem));
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot(shooterSubsystem));
        NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idle(shooterSubsystem));
        NamedCommands.registerCommand("AlignAndShoot",
            Commands.sequence(createAutoHubAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));

        autoChooser = AutoBuilder.buildAutoChooser("SimplePathAuto");
        SmartDashboard.putData("Auton Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Default drive — field-centric with auto-aim overlay
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double velocityX = -controller.getLeftY() * MaxSpeed;
                double velocityY = -controller.getLeftX() * MaxSpeed;

                if (autoAimEnabled && limelight.isTrackingHubTag()) {
                    double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
                    double targetHeading = currentHeading - Math.toRadians(limelight.getHorizontalOffset());

                    double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
                    double clampedOutput = MathUtil.clamp(rawOutput,
                        -VisionConstants.AUTO_AIM_MAX_ROTATION_RATE,
                         VisionConstants.AUTO_AIM_MAX_ROTATION_RATE);
                    double smoothOutput = autoAimSlew.calculate(clampedOutput);

                    double distance = limelight.getDistanceMeters();
                    if (distance > 0) {
                        shooterSubsystem.setVariableVelocity(
                            ShooterConstants.getVelocityForDistance(distance));
                    }

                    return drive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(smoothOutput);
                } else {
                    autoAimSlew.reset(0);
                    autoAimPID.reset();

                    if (shooterSubsystem.getState() == ShooterSubsystem.ShooterState.VARIABLE) {
                        shooterSubsystem.setVariableVelocity(0);
                    }

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

        controller.leftTrigger().whileTrue(ShooterCommands.ejectSequence(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));
    }

    public LimelightSubsystem getLimelight() { return limelight; }
    public ShooterSubsystem getShooter() { return shooterSubsystem; }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}