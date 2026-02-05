// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

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

  //--------------------DRIVETRAIN SETUP------------------------------------------------------------------------------------------------------------------------
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open loop voltage control over closed loop control
    
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

      ////--------------------------------------VISION SETUP--------------------------------------------------------------------------------------------------------------------------------------------------------------------\\\\
    // Limelight subsystem for AprilTag detection
    private final LimelightSubsystem limelight = new LimelightSubsystem();
    
    // Vision alignment command - automatically aligns robot with AprilTag 15 for scoring
      private final Limelight_Move alignToTag = new Limelight_Move(
        drivetrain, 
        limelight,
        () -> -controller.getLeftX()  
    );

   
  public LimelightSubsystem getLimelight() {
        return limelight;
    }


  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
    private final ShooterCommands shooterCommands = new ShooterCommands();

  public ShooterCommands getShooterCommands() {
    return shooterCommands;
  }



  private final SendableChooser<Command> autoChooser; // Autonomous command chooser

   
    public RobotContainer() {
       LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
       Limelight_Move limleightMove = new Limelight_Move(drivetrain, limelightSubsystem);

       ShooterSubsystem shooter = new ShooterSubsystem();
       
       PathPlannerAuto simplePathCommand = new PathPlannerAuto("SimplePathAuto");

        //NamedCommands.registerCommand("Test", Commands.print("Passed a test marker"));
        
        NamedCommands.registerCommand("Get Centered", alignToTag);
    
        autoChooser = AutoBuilder.buildAutoChooser("SimplePathAuto");
        SmartDashboard.putData("Auton Mode", autoChooser);
       
        configureBindings();
       
    }
    
        

        
            private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // ---- DRIVETRAIN BINDINGS -----------------------------------------------------------------------------------------------------------------------------
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) 
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) 
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

                 // ---- SYSID / FIELD-CENTRIC BINDINGS ----
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
 

        // reset the field-centric heading on right bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        //------------VISION ALIGNMENT------------------------------------------------------------------------------------------------------------------------
        
        //Hold START button to align with AprilTag 9
        controller.start().whileTrue(alignToTag);
        
         // reset the field-centric heading on right bumper press
        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // =====SHOOTER BINDINGS=====
        controller.rightTrigger().whileTrue(ShooterCommands.shoot(shooterSubsystem));

    }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
 
      public Command getAutonomousCommand() {
        return autoChooser.getSelected();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
}
