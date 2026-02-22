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

    // Feed rollers — push the ball into the flywheel
    private final TalonFX actFloor;
    private final TalonFX actCeiling;

    // Flywheel — spins the ball out of the shooter
    private final TalonFX flywheelMotor;

    // Feed uses DutyCycleOut (simple % power — no PID needed for a feed roller)
    private final DutyCycleOut feedRequest   = new DutyCycleOut(0);
    // Flywheel uses VelocityVoltage (needs consistent exit speed for accurate shots)
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest  = new NeutralOut();

    // Status signals — bulk refreshed once per loop to reduce CAN traffic
    private final StatusSignal<AngularVelocity> flywheelVelocitySig;
    private final StatusSignal<Voltage>         flywheelVoltageSig;
    private final StatusSignal<Current>         flywheelCurrentSig;

    // Feed roller state — what the floor and ceiling motors are doing
    public enum FeedState {
        IDLE,     // Stopped
        FEEDING,  // Pushing ball toward flywheel
        EJECTING, // Running in reverse to clear a stuck ball
        PASSING   // Running at pass power
    }

    // Flywheel state — what the flywheel motor is doing
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

    // Used only during VISION_TRACKING — updated every loop from Limelight distance
    private double variableVelocityRPS = 0;

    // For dashboard display
    private double lastCommandedFlywheelRPS = 0;

    public ShooterSubsystem() {
        actFloor      = new TalonFX(ShooterConstants.ACT_FLOOR,     ShooterConstants.CANIVORE);
        actCeiling    = new TalonFX(ShooterConstants.ACT_CEILING,   ShooterConstants.CANIVORE);
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR, ShooterConstants.CANIVORE);

        // Feed rollers — DutyCycleOut, no PID needed
        var feedConfig = new TalonFXConfiguration();
        feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        actFloor.getConfigurator().apply(feedConfig);
        actCeiling.getConfigurator().apply(feedConfig);

        // Flywheel — VelocityVoltage with PID for consistent exit speed
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_kV;
        flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelMotor.getConfigurator().apply(flywheelConfig);

        // Register signals — only flywheel needs velocity tracking
        flywheelVelocitySig = flywheelMotor.getVelocity();
        flywheelVoltageSig  = flywheelMotor.getMotorVoltage();
        flywheelCurrentSig  = flywheelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, flywheelVelocitySig);
        BaseStatusSignal.setUpdateFrequencyForAll(50,  flywheelVoltageSig, flywheelCurrentSig);
    }

    @Override
    public void periodic() {
        // Non-blocking fetch — gets latest values without stalling the loop
        BaseStatusSignal.waitForAll(0, flywheelVelocitySig, flywheelVoltageSig, flywheelCurrentSig);

        // Flywheel — VISION_TRACKING updates every loop since speed changes with distance.
        // All other states only update when the state changes, to reduce CAN traffic.
        if (flywheelState == FlywheelState.VISION_TRACKING) {
            setFlywheelVelocity(variableVelocityRPS);
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

        // Feed rollers — run independently from flywheel, only update on state change
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
        SmartDashboard.putNumber("Flywheel/Target RPS",     lastCommandedFlywheelRPS);
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

    /** Stop everything — feed and flywheel. */
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

    /** For teleop trigger — starts flywheel at full speed and feed simultaneously. */
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
    public void setVariableVelocity(double velocityRPS) {
        variableVelocityRPS = velocityRPS;
        if (flywheelState != FlywheelState.VISION_TRACKING) {
            flywheelState = FlywheelState.VISION_TRACKING;
        }
    }

    /** Called when auto-aim disengages. Returns flywheel to idle. */
    public void clearVariableVelocity() {
        if (flywheelState == FlywheelState.VISION_TRACKING) {
            variableVelocityRPS = 0;
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
            case VISION_TRACKING -> variableVelocityRPS;
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
        } else {
            actFloor.setControl(feedRequest.withOutput(power));
            actCeiling.setControl(feedRequest.withOutput(power));
        }
    }

    private void setFlywheelVelocity(double rps) {
        lastCommandedFlywheelRPS = rps;
        if (rps == 0.0) {
            flywheelMotor.setControl(neutralRequest);
        } else {
            flywheelMotor.setControl(flywheelRequest.withVelocity(rps));
        }
    }
}
