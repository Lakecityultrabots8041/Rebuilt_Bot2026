package frc.robot.subsystems.shoot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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

    // Feed rollers push the ball toward the flywheel
    private final TalonFX feedFloor;
    private final TalonFX feedCeiling;
    private final TalonFX lo4der; // 12:1 gearbox, pulls ball up into flywheel

    // Flywheel launches the ball
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelMotor2;

    // Feed rollers use % power (no PID), flywheel uses closed-loop velocity
    private final DutyCycleOut feedRequest      = new DutyCycleOut(0);
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0);
    private final NeutralOut neutralRequest      = new NeutralOut();

    // Flywheel telemetry, refreshed once per loop in periodic()
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

    private FeedState lo4dState = FeedState.IDLE;
    private FeedState lastLo4dState = null;

    private FlywheelState flywheelState     = FlywheelState.IDLE;
    private FlywheelState lastFlywheelState = null;

    private double distanceSpeed = 0;       // RPS picked from Limelight distance table
    private double flywheelTargetSpeed = 0; // Last speed sent to motor, shown on dashboard

    public ShooterSubsystem() {
        feedFloor     = new TalonFX(ShooterConstants.FEED_FLOOR);
        feedCeiling   = new TalonFX(ShooterConstants.FEED_CEILING);
        lo4der        = new TalonFX(ShooterConstants.LO4D3R);
        flywheelMotor = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR);
        flywheelMotor2 = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR2);

        // Feed rollers: direct drive, no PID needed
        var feedConfig = new TalonFXConfiguration();
        feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feedConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        feedConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feedConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FEED_SUPPLY_CURRENT_LIMIT;
        feedFloor.getConfigurator().apply(feedConfig);
        feedCeiling.getConfigurator().apply(feedConfig);

        // Lo4d3r: 12:1 gearbox, 4 belts
        var lo4derConfig = new TalonFXConfiguration();
        lo4derConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lo4derConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        lo4derConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lo4derConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.LO4D3R_SUPPLY_CURRENT_LIMIT;
        lo4der.getConfigurator().apply(lo4derConfig);

        // Flywheel: PID velocity control so exit speed is consistent
        var flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = ShooterConstants.FLYWHEEL_kP;
        flywheelConfig.Slot0.kV = ShooterConstants.FLYWHEEL_kV;
        flywheelConfig.Slot0.kS = ShooterConstants.FLYWHEEL_kS;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        flywheelConfig.Audio.AllowMusicDurDisable = true;
        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.getConfigurator().apply(flywheelConfig);

        // Flywheel telemetry for dashboard + ready checks
        flywheelVelocitySig = flywheelMotor.getVelocity();
        flywheelVoltageSig  = flywheelMotor.getMotorVoltage();
        flywheelCurrentSig  = flywheelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, flywheelVelocitySig);
        BaseStatusSignal.setUpdateFrequencyForAll(50,  flywheelVoltageSig, flywheelCurrentSig);

        // Only keep the signals we actually read, disable everything else
        flywheelMotor.optimizeBusUtilization();
        flywheelMotor2.optimizeBusUtilization();
        feedFloor.optimizeBusUtilization();
        feedCeiling.optimizeBusUtilization();
        lo4der.optimizeBusUtilization();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.waitForAll(0, flywheelVelocitySig, flywheelVoltageSig, flywheelCurrentSig);

        // VISION_TRACKING updates every loop because distance changes. Other states only on change.
        if (flywheelState == FlywheelState.VISION_TRACKING) {
            setFlywheelVelocity(distanceSpeed);
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

        if (feedState != lastFeedState) {
            switch (feedState) { 
                case IDLE     -> setFeedPower(0.0);
                case FEEDING  -> setFeedPower(ShooterConstants.FEED_POWER);
                case EJECTING -> setFeedPower(ShooterConstants.EJECT_POWER);
                case PASSING  -> setFeedPower(ShooterConstants.PASS_POWER);
            }
            lastFeedState = feedState;
        }

        if (lo4dState != lastLo4dState) {
            switch (lo4dState) {
                case IDLE     -> setLo4derPower(0.0);
                case FEEDING  -> setLo4derPower(ShooterConstants.LO4D3R_POWER);
                case EJECTING -> setLo4derPower(ShooterConstants.LO4D3R_EJECT_POWER);
                case PASSING  -> setLo4derPower(ShooterConstants.LO4D3R_POWER);
            }
            lastLo4dState = lo4dState;
         }

        SmartDashboard.putString("Shooter/Feed State",      feedState.toString());
        SmartDashboard.putString("Shooter/Lo4d State",      lo4dState.toString());
        SmartDashboard.putString("Shooter/Flywheel State",  flywheelState.toString());
        SmartDashboard.putNumber("Flywheel/Actual RPS",     flywheelVelocitySig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Target RPS",     flywheelTargetSpeed);
        SmartDashboard.putNumber("Flywheel/Voltage",        flywheelVoltageSig.getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Current",        flywheelCurrentSig.getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/Flywheel Ready", isFlywheelReady());
    }

    // ===== COMMAND FACTORIES =====
    // Each just flips a state enum. periodic() handles the actual motor writes.

    public Command revFlywheel()  { return runOnce(() -> flywheelState = FlywheelState.REVVING).withName("RevFlywheel"); }
    public Command readyFlywheel(){ return runOnce(() -> flywheelState = FlywheelState.READY).withName("ReadyFlywheel"); }
    public Command startFeeding() { return runOnce(() -> feedState = FeedState.FEEDING).withName("StartFeeding"); }
    public Command startLo4d()    { return runOnce(() -> lo4dState = FeedState.FEEDING).withName("StartLo4d"); }
    public Command stopFeed()     { return runOnce(() -> feedState = FeedState.IDLE).withName("StopFeed"); }
    public Command stopLo4d()     { return runOnce(() -> lo4dState = FeedState.IDLE).withName("StopLo4d"); }
    public Command ejectFeed()    { return runOnce(() -> feedState = FeedState.EJECTING).withName("EjectFeed"); }
    public Command ejectLo4d()    { return runOnce(() -> lo4dState = FeedState.EJECTING).withName("EjectLo4d"); }

    public Command idleAll() {
        return runOnce(() -> {
            feedState     = FeedState.IDLE;
            lo4dState     = FeedState.IDLE;
            flywheelState = FlywheelState.IDLE;
        }).withName("IdleAll");
    }

    public Command passAll() {
        return runOnce(() -> {
            flywheelState = FlywheelState.PASSING;
            feedState     = FeedState.PASSING;
            lo4dState     = FeedState.PASSING;
        }).withName("PassAll");
    }

    // Fixed speed shoot: flywheel + feed start together at FLYWHEEL_READY_RPS
    public Command shoot() {
        return runOnce(() -> {
            flywheelState = FlywheelState.READY;
            feedState     = FeedState.FEEDING;
            lo4dState     = FeedState.FEEDING;
        }).withName("Shoot");
    }

    // Distance mode shoot: picks flywheel RPS from Limelight distance each loop.
    // Falls back to fixed FLYWHEEL_READY_RPS when distance mode is off or no target.
    public Command shootWithDistance(DoubleSupplier distanceMeters, BooleanSupplier useDistance) {
        return run(() -> {
            double distance = distanceMeters.getAsDouble();
            if (useDistance.getAsBoolean() && distance > 0) {
                distanceSpeed = ShooterConstants.getSpeedForDistance(distance);
                flywheelState = FlywheelState.VISION_TRACKING;
            } else if (flywheelState != FlywheelState.READY) {
                flywheelState = FlywheelState.READY;
            }
            feedState = FeedState.FEEDING;
            lo4dState = FeedState.FEEDING;
        }).withName("ShootWithDistance");
    }

    // Short aliases for NamedCommands and RobotContainer
    public Command revUp()  { return revFlywheel(); }
    public Command idle()   { return idleAll();     }
    public Command eject()  { return ejectFeed();   }
    public Command pass()   { return passAll();     }

    // ===== READY CHECKS =====

    public boolean isFlywheelReady() {
        double target = switch (flywheelState) {
            case IDLE            -> ShooterConstants.FLYWHEEL_IDLE_RPS;
            case REVVING         -> ShooterConstants.FLYWHEEL_REV_RPS;
            case READY           -> ShooterConstants.FLYWHEEL_READY_RPS;
            case PASSING         -> ShooterConstants.FLYWHEEL_PASS_RPS;
            case VISION_TRACKING -> distanceSpeed;
        };
        double actual = flywheelVelocitySig.getValueAsDouble();
        return Math.abs(actual - target) < ShooterConstants.FLYWHEEL_TOLERANCE_RPS;
    }

    public boolean atTargetVelocity()        { return isFlywheelReady(); }
    public boolean atFlywheelTargetVelocity() { return isFlywheelReady(); }

    public Command waitUntilFlywheelReady() {
        return Commands.waitUntil(this::isFlywheelReady)
                .withTimeout(ShooterConstants.READY_TIMEOUT_SECONDS)
                .withName("WaitForFlywheel");
    }

    public Command waitUntilReady() { return waitUntilFlywheelReady(); }

    // ===== GETTERS =====
    public FeedState     getFeedState()     { return feedState;     }
    public FlywheelState getFlywheelState() { return flywheelState; }

    // SimManager needs these to feed physics back into the motor controllers
    public TalonFX getFlywheelMotor()  { return flywheelMotor;  }
    public TalonFX getFlywheelMotor2() { return flywheelMotor2; }

    // ===== PRIVATE HELPERS =====

    private void setFeedPower(double power) {
        if (power == 0.0) {
            feedFloor.setControl(neutralRequest);
            feedCeiling.setControl(neutralRequest);
        } else {
            // Negated because ceiling motor faces the opposite direction
            feedFloor.setControl(feedRequest.withOutput(-power));
            feedCeiling.setControl(feedRequest.withOutput(-power));
        }
    }

    private void setLo4derPower(double power) {
        if (power == 0.0) {
            lo4der.setControl(neutralRequest);
        } else {
            lo4der.setControl(feedRequest.withOutput(power));
        }
    }

    private void setFlywheelVelocity(double rps) {
        flywheelTargetSpeed = rps;
        if (rps == 0.0) {
            flywheelMotor.setControl(neutralRequest);
            flywheelMotor2.setControl(neutralRequest);
        } else {
            flywheelMotor.setControl(flywheelRequest.withVelocity(rps));
            flywheelMotor2.setControl(flywheelRequest.withVelocity(rps));
        }
    }
}
