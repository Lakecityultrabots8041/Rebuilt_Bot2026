package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    // The Blinkin is controlled like a Spark motor controller over PWM.
    // Sending a value between -1.0 and 1.0 selects a built-in pattern.
    private final Spark blinkin;

    // Conditions checked every loop to decide what pattern to show.
    // These are passed in from RobotContainer so LED logic stays in one file.
    private final BooleanSupplier visionAlignedSupplier;
    private final BooleanSupplier autoAimActiveSupplier;
    private final BooleanSupplier intakingSupplier;

    // Alliance tracking for idle/disabled colors
    private Alliance lastAlliance = null;
    private double disabledValue = LEDConstants.DEFAULT_DISABLED;
    private double idleValue     = LEDConstants.DEFAULT_IDLE;

    private LEDState currentState = LEDState.IDLE;
    private LEDState lastState    = null;

    // Highest priority at the top, lowest at the bottom.
    // The first condition that is true wins.
    public enum LEDState {
        VISION_ALIGNED,   // Auto-aim on AND locked on tag
        AUTO_AIM_ACTIVE,  // Auto-aim toggle on, searching
        INTAKING,         // Intake rollers running
        AUTONOMOUS,       // Running an auto routine
        DISABLED,         // Robot disabled
        IDLE              // Nothing happening
    }

    public LEDSubsystem(
            BooleanSupplier visionAlignedSupplier,
            BooleanSupplier autoAimActiveSupplier,
            BooleanSupplier intakingSupplier) {

        this.visionAlignedSupplier = visionAlignedSupplier;
        this.autoAimActiveSupplier = autoAimActiveSupplier;
        this.intakingSupplier      = intakingSupplier;

        blinkin = new Spark(LEDConstants.PWM_PORT);
    }

    @Override
    public void periodic() {
        updateAlliancePatterns();

        // Priority waterfall: first match wins
        LEDState newState;
        if (visionAlignedSupplier.getAsBoolean()) {
            newState = LEDState.VISION_ALIGNED;
        } else if (autoAimActiveSupplier.getAsBoolean()) {
            newState = LEDState.AUTO_AIM_ACTIVE;
        } else if (intakingSupplier.getAsBoolean()) {
            newState = LEDState.INTAKING;
        } else if (DriverStation.isAutonomousEnabled()) {
            newState = LEDState.AUTONOMOUS;
        } else if (DriverStation.isDisabled()) {
            newState = LEDState.DISABLED;
        } else {
            newState = LEDState.IDLE;
        }

        // Only send PWM on state change to avoid redundant writes
        if (newState != lastState) {
            blinkin.set(getPatternValue(newState));
            SmartDashboard.putString("LED/State", newState.toString());
            currentState = newState;
            lastState = newState;
        }
    }

    private double getPatternValue(LEDState state) {
        return switch (state) {
            case VISION_ALIGNED  -> LEDConstants.VISION_ALIGNED;
            case AUTO_AIM_ACTIVE -> LEDConstants.AUTO_AIM_ACTIVE;
            case INTAKING        -> LEDConstants.INTAKING;
            case AUTONOMOUS      -> LEDConstants.AUTONOMOUS;
            case DISABLED        -> disabledValue;
            case IDLE            -> idleValue;
        };
    }

    // Update alliance colors when FMS connects and tells us red or blue
    private void updateAlliancePatterns() {
        Alliance current = DriverStation.getAlliance().orElse(null);
        if (current != lastAlliance) {
            if (current == Alliance.Red) {
                disabledValue = LEDConstants.DISABLED_RED;
                idleValue     = LEDConstants.IDLE_RED;
            } else {
                disabledValue = LEDConstants.DISABLED_BLUE;
                idleValue     = LEDConstants.IDLE_BLUE;
            }
            lastAlliance = current;

            // Force a re-send if we are currently showing an alliance pattern
            if (currentState == LEDState.DISABLED || currentState == LEDState.IDLE) {
                lastState = null;
            }
        }
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}
