package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// See docs/LED_GUIDE.md for priority, shift timing, and how to change patterns.
public class LEDSubsystem extends SubsystemBase {

    private final Spark blinkin;

    private final BooleanSupplier visionAlignedSupplier;
    private final BooleanSupplier autoAimActiveSupplier;
    private final BooleanSupplier intakingSupplier;

    private Alliance lastAlliance = null;
    private double disabledValue = LEDConstants.DEFAULT_DISABLED;

    private LEDState currentState = LEDState.IDLE;
    private LEDState lastState    = null;

    // Priority order: first enum = highest priority. See LED_GUIDE.md for details.
    public enum LEDState {
        VISION_ALIGNED,
        AUTO_AIM_ACTIVE,
        INTAKING,
        AUTONOMOUS,
        DISABLED,
        END_GAME,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        IDLE
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
        } else if (DriverStation.isTeleopEnabled()) {
            newState = getTeleopShiftState();
        } else {
            newState = LEDState.IDLE;
        }

        // Only send PWM on state change
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
            case END_GAME        -> LEDConstants.END_GAME;
            case SHIFT_1         -> LEDConstants.SHIFT_1;
            case SHIFT_2         -> LEDConstants.SHIFT_2;
            case SHIFT_3         -> LEDConstants.SHIFT_3;
            case SHIFT_4         -> LEDConstants.SHIFT_4;
            case DISABLED        -> disabledValue;
            case IDLE            -> LEDConstants.IDLE;
        };
    }

    // Boundaries match Robot.java shift tracking
    private LEDState getTeleopShiftState() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0) return LEDState.IDLE;

        if (matchTime <= 30)  return LEDState.END_GAME;
        if (matchTime <= 55)  return LEDState.SHIFT_4;
        if (matchTime <= 80)  return LEDState.SHIFT_3;
        if (matchTime <= 105) return LEDState.SHIFT_2;
        if (matchTime <= 130) return LEDState.SHIFT_1;

        return LEDState.IDLE;
    }

    private void updateAlliancePatterns() {
        Alliance current = DriverStation.getAlliance().orElse(null);
        if (current != lastAlliance) {
            if (current == Alliance.Red) {
                disabledValue = LEDConstants.DISABLED_RED;
            } else {
                disabledValue = LEDConstants.DISABLED_BLUE;
            }
            lastAlliance = current;

            // Force re-send if currently showing disabled pattern
            if (currentState == LEDState.DISABLED) {
                lastState = null;
            }
        }
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}
