// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Limelight_Move;
import frc.robot.generated.TunerConstants;

import frc.robot.commands.ShooterCommands;




@SuppressWarnings("unused")

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    //Setup Command Xbox Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    //--------------------DRIVETRAIN SETUP--------------------------------------------------------
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //--------------------------------------VISION SETUP------------------------------------------
    // Limelight subsystem for AprilTag detection
    private final LimelightSubsystem limelight = new LimelightSubsystem();
    
    // Vision alignment command - automatically aligns robot with AprilTag
    private final Limelight_Move alignToTag = new Limelight_Move(
        drivetrain, 
        limelight,
        () -> -controller.getLeftX()  
    );

    //--------------------------------------SHOOTER SETUP-----------------------------------------
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
     

    //--------------------------------------AUTO SETUP--------------------------------------------
    private final SendableChooser<Command> autoChooser;

   
    public RobotContainer() {
        // Register named commands for PathPlanner
        NamedCommands.registerCommand("Get Centered", alignToTag);
        NamedCommands.registerCommand("Rev Shooter", ShooterCommands.revUp(shooterSubsystem));
        NamedCommands.registerCommand("Shoot", ShooterCommands.shoot(shooterSubsystem));
        NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idle(shooterSubsystem));
    
        autoChooser = AutoBuilder.buildAutoChooser("SimplePathAuto");
        SmartDashboard.putData("Auton Mode", autoChooser);
       
        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        // ---- DRIVETRAIN BINDINGS ----
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) 
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate)
            )
        );

        // ---- SYSID BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // Reset field-centric heading on right bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // ---- VISION ALIGNMENT ----
        // Hold START button to align with AprilTag
        controller.start().whileTrue(alignToTag);

        // ---- SHOOTER BINDINGS ----
        // Simple version - just using the basic commands
        controller.rightTrigger().onTrue(ShooterCommands.shootSequence(shooterSubsystem));
        
        // Additional shooter controls (optional - comment out if you don't want them)
        // controller.leftTrigger().whileTrue(ShooterCommands.revUp(shooterSubsystem));
        // controller.a().onTrue(ShooterCommands.idle(shooterSubsystem));
        // controller.b().onTrue(ShooterCommands.eject(shooterSubsystem));
    }

    // ---- ACCESSOR METHODS ----
    public LimelightSubsystem getLimelight() {
        return limelight;
    }

    public ShooterSubsystem getShooter() {
        return shooterSubsystem;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}