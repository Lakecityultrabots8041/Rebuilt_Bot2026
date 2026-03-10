package frc.robot;

import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shoot.ShooterSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystems;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.commands.Limelight_Move;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.DriveConstants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

    private final CommandXboxController controller = new CommandXboxController(0);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final LimelightSubsystem limelightShooter = new LimelightSubsystem("limelight-shooter", false);
    private final LimelightSubsystem limelightIntake = new LimelightSubsystem("limelight-intake", true);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final IntakeSubsystems intakeSubsystem = new IntakeSubsystems();
    private final LEDSubsystem ledSubsystem;

    // Tracked commanded speeds for asymmetric slew
    private double currentDriveX = 0.0;
    private double currentDriveY = 0.0;

    // Back button toggles between distance-based and fixed speed shooting.
    // Turn off before comp if the distance table isn't tuned.
    private boolean useDistanceShot = true;

    private final SendableChooser<Command> autoChooser;

    // Teleop alignment factories, each call returns a new instance
    private Limelight_Move createHubAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getHubTags, () -> -controller.getLeftX());
    }

    /* private Limelight_Move createTowerAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getTowerTags, () -> -controller.getLeftX());
    }

    private Limelight_Move createOutpostAlign() {
        return new Limelight_Move(drivetrain, limelightShooter,
            VisionConstants::getOutpostTags, () -> -controller.getLeftX());
    }
   */

    // Auto alignment factories (no driver strafe input)
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
        // Both cameras fuse poses into the drivetrain Kalman filter
        limelightShooter.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelightShooter.setVisionPoseConsumer(
            (pose, timestamp, accuracy) -> drivetrain.addVisionMeasurement(pose, timestamp, accuracy));
        limelightShooter.setSpinRateSupplier(
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

        limelightIntake.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
        limelightIntake.setVisionPoseConsumer(
            (pose, timestamp, accuracy) -> drivetrain.addVisionMeasurement(pose, timestamp, accuracy));
        limelightIntake.setSpinRateSupplier(
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond);

        // PathPlanner named commands
        NamedCommands.registerCommand("Align Hub", createAutoHubAlign());
        NamedCommands.registerCommand("Align Outpost", createAutoOutpostAlign());
        NamedCommands.registerCommand("Align Tower", createAutoTowerAlign());
        NamedCommands.registerCommand("Align Trench", createAutoTrenchAlign());

        NamedCommands.registerCommand("Rev Flywheel", ShooterCommands.revUpFlywheel(shooterSubsystem));
        NamedCommands.registerCommand("Shoot", ShooterCommands.quickShoot(shooterSubsystem));
        NamedCommands.registerCommand("Idle Shooter", ShooterCommands.idle(shooterSubsystem));
        NamedCommands.registerCommand("Pass", ShooterCommands.passSequence(shooterSubsystem));
        NamedCommands.registerCommand("Quick Shoot", ShooterCommands.quickShoot(shooterSubsystem));
        NamedCommands.registerCommand("AlignHubAndShoot",
            Commands.sequence(createAutoHubAlign().andThen(ShooterCommands.quickShoot(shooterSubsystem))));
        NamedCommands.registerCommand("AlignOutpostAndShoot",
            Commands.sequence(createAutoOutpostAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));
        NamedCommands.registerCommand("AlignTowerAndShoot",
            Commands.sequence(createAutoTowerAlign().andThen(ShooterCommands.shootSequence(shooterSubsystem))));

        NamedCommands.registerCommand("Intake", IntakeCommands.autonIntake(intakeSubsystem));
        NamedCommands.registerCommand("Eject", IntakeCommands.eject(intakeSubsystem));
        NamedCommands.registerCommand("Idle Intake", IntakeCommands.idle(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Stow", IntakeCommands.pivotToStow(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Intake", IntakeCommands.pivotToIntake(intakeSubsystem));
        NamedCommands.registerCommand("Pivot To Travel", IntakeCommands.pivotToTravel(intakeSubsystem));

        // LEDs react to subsystem states, no commands needed
        ledSubsystem = new LEDSubsystem(
            () -> limelightShooter.isTrackingHubTag(),
            () -> useDistanceShot && limelightShooter.hasValidTarget(),
            () -> intakeSubsystem.getState() == IntakeSubsystems.IntakeState.INTAKING
        );

        autoChooser = AutoBuilder.buildAutoChooser("SimplePath Auton");
        SmartDashboard.putData("Auton Mode", autoChooser);
        SmartDashboard.putBoolean("Shooter/Distance Mode", useDistanceShot);

        configureBindings();
    }

    private void configureBindings() {
        // Field-centric drive with asymmetric slew
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                currentDriveX = applyDriveSlew(currentDriveX, -controller.getLeftY() * MaxSpeed * 0.9);
                currentDriveY = applyDriveSlew(currentDriveY, -controller.getLeftX() * MaxSpeed * 0.9);

                return drive.withVelocityX(currentDriveX)
                            .withVelocityY(currentDriveY)
                            .withRotationalRate(-controller.getRightX() * MaxAngularRate);
            })
        );

        controller.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Vision alignment: Start = hub
        controller.start().whileTrue(createHubAlign());

        // Back = toggle distance-based vs fixed speed shooting
        controller.back().onTrue(Commands.runOnce(() -> {
            useDistanceShot = !useDistanceShot;
            SmartDashboard.putBoolean("Shooter/Distance Mode", useDistanceShot);
        }));

        // Trigger shoots. Distance mode picks speed from Limelight, fixed mode uses 43 RPS.
        controller.rightTrigger().whileTrue(
            shooterSubsystem.shootWithDistance(
                () -> limelightShooter.getDistanceMeters(),
                () -> useDistanceShot))
            .onFalse(ShooterCommands.idle(shooterSubsystem));

        controller.x().whileTrue(ShooterCommands.eject(shooterSubsystem))
            .onFalse(ShooterCommands.idle(shooterSubsystem));

        // Left trigger = intake
        controller.leftTrigger().whileTrue(IntakeCommands.intake(intakeSubsystem))
            .onFalse(IntakeCommands.idle(intakeSubsystem));

        // Pivot presets: Y = stow, A = intake, B = travel
        controller.y().onTrue(IntakeCommands.pivotToStow(intakeSubsystem));
        controller.a().onTrue(IntakeCommands.pivotToIntake(intakeSubsystem));
        controller.b().onTrue(IntakeCommands.pivotToTravel(intakeSubsystem));
    }

    // Fast accel, controlled decel. Tune in DriveConstants.
    private double applyDriveSlew(double current, double requested) {
        double rateLimit = (Math.abs(requested) > Math.abs(current))
            ? DriveConstants.MAX_TELEOP_ACCEL
            : DriveConstants.MAX_TELEOP_DECEL;
        double maxDelta = rateLimit * 0.02;
        return current + MathUtil.clamp(requested - current, -maxDelta, maxDelta);
    }

    public void updateDriverDashboard() {
        boolean shooterReady = shooterSubsystem.atTargetVelocity();
        boolean trackingTarget = limelightShooter.isTrackingHubTag();
        boolean intakeDown = intakeSubsystem.getPivotState() == IntakeSubsystems.PivotState.INTAKE
                             && intakeSubsystem.isPivotAtTarget();

        SmartDashboard.putBoolean("Driver/Shooter Ready",   shooterReady);
        SmartDashboard.putBoolean("Driver/Tracking Target",  trackingTarget);
        SmartDashboard.putBoolean("Driver/Distance Mode",    useDistanceShot);
        SmartDashboard.putBoolean("Driver/Intake Down",      intakeDown);
    }

    public LimelightSubsystem getLimelightShooter() { return limelightShooter; }
    public LimelightSubsystem getLimelightIntake() { return limelightIntake; }
    public ShooterSubsystem getShooter() { return shooterSubsystem; }
    public IntakeSubsystems getIntake() { return intakeSubsystem; }
    public LEDSubsystem getLEDs() { return ledSubsystem; }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
