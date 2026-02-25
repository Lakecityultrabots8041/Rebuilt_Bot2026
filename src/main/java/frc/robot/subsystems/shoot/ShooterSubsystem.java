package frc.robot.subsystems.shoot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    // Feed rollers push the ball into the flywheel
    private final TalonFX actFloor;
    private final TalonFX actCeiling;
    private final TalonFX actUpper; // Pulls ball from floor up into flywheel

    // Flywheel launches the ball
    private final TalonFX flywheelMotor;

    // Feed uses DutyCycleOut (simple % power, no PID)
    private final DutyCycleOut feedRequest   = new DutyCycleOut(0);
    // Flywheel uses VelocityVoltage (PID for consistent exit speed)
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest  = new NeutralOut();

    // Status signals, bulk refreshed once per loop
    private final StatusSignal<AngularVelocity> flywheelVelocitySig;
    private final StatusSignal<Voltage>         flywheelVoltageSig;
    private final StatusSignal<Current>         flywheelCurrentSig;

    public enum FeedState {
        IDLE,     // Stopped
        FEEDING,  // Pushing ball toward flywheel
        EJECTING, // Running in reverse to clear a stuck ball
        PASSING   // Running at pass power
    }

    public enum FlywheelState {
        IDLE,           // Stopped
        REVVING,        // Pre-spinning before full speed
        READY,          // At full shooting speed
        PASSING,        // At passing speed
        VISION_TRACKING // Speed set by Limelight distance, updated every loop
    }

    private FeedState     feedState     = FeedState.IDLE;
    private FeedState     lastFeedState = null;

    private FlywheelState flywheelState     = FlywheelState.IDLE;
    private FlywheelState lastFlywheelState = null;

    // Flywheel speed set by auto-aim, updated every loop from Limelight distance
    private double autoAimSpeed = 0;

    // Last speed sent to the flywheel for dashboard display
    private double flywheelTargetSpeed = 0;

    public ShooterSubsystem() {
        actFloor      = new TalonFX(ShooterConstants.ACT_FLOOR,     ShooterConstants.CANIVORE);
        actCeiling    = new TalonFX(ShooterConstants.ACT_CEILING,   ShooterConstants.CANIVORE);
        actUpper      = new TalonFX(ShooterConstants.ACT_UPPER,     ShooterConstants.CANIVORE);
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR, ShooterConstants.CANIVORE);

        // Feed rollers (floor + ceiling): DutyCycleOut, no PID needed
        var feedConfig = new TalonFXConfiguration();
        feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        feedConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FEED_STATOR_CURRENT_LIMIT;
        feedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feedConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FEED_SUPPLY_CURRENT_LIMIT;
        actFloor.getConfigurator().apply(feedConfig);
        actCeiling.getConfigurator().apply(feedConfig);

        // Upper feed roller (motor #8, 12:1 gearbox) - new motor, factory default
        // first to clear any leftover config that could block output.
        var upperConfig = new TalonFXConfiguration();
        upperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        upperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        upperConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.UPPER_STATOR_CURRENT_LIMIT;
        upperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.UPPER_SUPPLY_CURRENT_LIMIT;
        actUpper.getConfigurator().apply(upperConfig);

        // Flywheel: VelocityVoltage with PID for consistent exit speed
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_kV;
        flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        flywheelMotor.getConfigurator().apply(flywheelConfig);

        // Register flywheel signals for velocity tracking
        flywheelVelocitySig = flywheelMotor.getVelocity();
        flywheelVoltageSig  = flywheelMotor.getMotorVoltage();
        flywheelCurrentSig  = flywheelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, flywheelVelocitySig);
        BaseStatusSignal.setUpdateFrequencyForAll(50,  flywheelVoltageSig, flywheelCurrentSig);
    }

    @Override
    public void periodic() {
        // Non-blocking fetch of latest values
        BaseStatusSignal.waitForAll(0, flywheelVelocitySig, flywheelVoltageSig, flywheelCurrentSig);

        // VISION_TRACKING updates every loop. Other states only update on change.
        if (flywheelState == FlywheelState.VISION_TRACKING) {
            setFlywheelVelocity(autoAimSpeed);
        } else if (flywheelState != lastFlywheelState) {
            switch (flywheelState) {
                case IDLE    -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_IDLE_RPS);
                case REVVING -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_REV_RPS);
                case READY   -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_READY_RPS);
                case PASSING -> setFlywheelVelocity(ShooterConstants.FLYWHEEL_PASS_RPS);
                default      -> {}
            }
            lastFlywheelState = flywheelState;
        }

        // Feed rollers only update on state change
        if (feedState != lastFeedState) {
            switch (feedState) {
                case IDLE     -> setFeedPower(0.0);
                case FEEDING  -> setFeedPower(ShooterConstants.FEED_POWER);
                case EJECTING -> setFeedPower(ShooterConstants.EJECT_POWER);
                case PASSING  -> setFeedPower(ShooterConstants.PASS_POWER);
            }
            lastFeedState = feedState;
        }

        // Dashboard
        SmartDashboard.putString("Shooter/Feed State",      feedState.toString());
        SmartDashboard.putString("Shooter/Flywheel State",  flywheelState.toString());
        SmartDashboard.putNumber("Flywheel/Actual RPS",     flywheelVelocitySig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Target RPS",     flywheelTargetSpeed);
        SmartDashboard.putNumber("Flywheel/Voltage",        flywheelVoltageSig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Current",        flywheelCurrentSig.getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/Flywheel Ready", isFlywheelReady());
    }

    // ===== COMMAND FACTORIES =====

    /** Spin flywheel up to pre-rev speed. Used before going to full READY. */
    public Command revFlywheel() {
        return runOnce(() -> flywheelState = FlywheelState.REVVING)
                .withName("RevFlywheel");
    }

    /** Spin flywheel to full shooting speed. Feed rollers stay off. */
    public Command readyFlywheel() {
        return runOnce(() -> flywheelState = FlywheelState.READY)
                .withName("ReadyFlywheel");
    }

    /** Turn feed rollers on. Call this AFTER flywheel is at speed. */
    public Command startFeeding() {
        return runOnce(() -> feedState = FeedState.FEEDING)
                .withName("StartFeeding");
    }

    /** Turn feed rollers off. Flywheel keeps spinning. */
    public Command stopFeed() {
        return runOnce(() -> feedState = FeedState.IDLE)
                .withName("StopFeed");
    }

    /** Stop feed and flywheel. */
    public Command idleAll() {
        return runOnce(() -> {
            feedState     = FeedState.IDLE;
            flywheelState = FlywheelState.IDLE;
        }).withName("IdleAll");
    }

    /** Run feed rollers in reverse to clear a stuck ball. Flywheel state unchanged. */
    public Command ejectFeed() {
        return runOnce(() -> feedState = FeedState.EJECTING)
                .withName("EjectFeed");
    }

    /** Set flywheel and feed to passing speed/power. */
    public Command passAll() {
        return runOnce(() -> {
            flywheelState = FlywheelState.PASSING;
            feedState     = FeedState.PASSING;
        }).withName("PassAll");
    }

    // ===== BACKWARD-COMPATIBLE METHODS =====
    // These keep RobotContainer and NamedCommands working without changes.

    /** Teleop trigger: starts flywheel and feed at once. */
    public Command shoot() {
        return runOnce(() -> {
            flywheelState = FlywheelState.READY;
            feedState     = FeedState.FEEDING;
        }).withName("Shoot");
    }

    public Command revUp()  { return revFlywheel(); }
    public Command idle()   { return idleAll();     }
    public Command eject()  { return ejectFeed();   }
    public Command pass()   { return passAll();     }

    // ===== VISION TRACKING =====

    /**
     * Called each loop by auto-aim to set flywheel speed based on Limelight distance.
     * Feed rollers are controlled separately by the driver's trigger.
     */
    public void setAutoAimSpeed(double speed) {
        autoAimSpeed = speed;
        if (flywheelState != FlywheelState.VISION_TRACKING) {
            flywheelState = FlywheelState.VISION_TRACKING;
        }
    }

    /** Called when auto-aim disengages. Returns flywheel to idle. */
    public void clearAutoAimSpeed() {
        if (flywheelState == FlywheelState.VISION_TRACKING) {
            autoAimSpeed = 0;
            flywheelState       = FlywheelState.IDLE;
            lastFlywheelState   = null;
        }
    }

    // ===== READY CHECKS =====

    /** True when flywheel is within tolerance of its target speed. */
    public boolean isFlywheelReady() {
        double target = switch (flywheelState) {
            case IDLE            -> ShooterConstants.FLYWHEEL_IDLE_RPS;
            case REVVING         -> ShooterConstants.FLYWHEEL_REV_RPS;
            case READY           -> ShooterConstants.FLYWHEEL_READY_RPS;
            case PASSING         -> ShooterConstants.FLYWHEEL_PASS_RPS;
            case VISION_TRACKING -> autoAimSpeed;
        };
        double actual = flywheelVelocitySig.getValueAsDouble();
        return Math.abs(actual - target) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS;
    }

    // Kept for backward compatibility with RobotContainer
    public boolean atTargetVelocity()        { return isFlywheelReady(); }
    public boolean atFlywheelTargetVelocity() { return isFlywheelReady(); }

    /** Waits until flywheel reaches its target speed, or times out. */
    public Command waitUntilFlywheelReady() {
        return Commands.waitUntil(this::isFlywheelReady)
                .withTimeout(ShooterConstants.READY_TIMEOUT_SECONDS)
                .withName("WaitForFlywheel");
    }

    // Kept for backward compatibility with ShooterCommands
    public Command waitUntilReady() { return waitUntilFlywheelReady(); }

    // ===== GETTERS =====
    public FeedState     getFeedState()     { return feedState;     }
    public FlywheelState getFlywheelState() { return flywheelState; }

    // ===== PRIVATE HELPERS =====

    private void setFeedPower(double power) {
        if (power == 0.0) {
            actFloor.setControl(neutralRequest);
            actCeiling.setControl(neutralRequest);
            actUpper.setControl(neutralRequest);
        } else {
            actFloor.setControl(feedRequest.withOutput(power));
            actCeiling.setControl(feedRequest.withOutput(-power)); // Negated because ceiling faces floor
            actUpper.setControl(feedRequest.withOutput(-power));   // Negated, pulls ball upward like ceiling
        }
    }

    private void setFlywheelVelocity(double rps) {
        flywheelTargetSpeed = rps;
        if (rps == 0.0) {
            flywheelMotor.setControl(neutralRequest);
        } else {
            flywheelMotor.setControl(flywheelRequest.withVelocity(rps));
        }
    }
}
