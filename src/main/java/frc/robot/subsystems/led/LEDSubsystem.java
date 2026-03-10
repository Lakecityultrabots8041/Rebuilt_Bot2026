package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// See docs/LED_GUIDE.md for priority, shift timing, and how to change patterns.
public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle;

    private final BooleanSupplier visionAlignedSupplier;
    private final BooleanSupplier autoAimActiveSupplier;
    private final BooleanSupplier intakingSupplier;

    private Alliance lastAlliance = null;
    private RGBWColor allianceColor = LEDConstants.BLUE;

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

        candle = new CANdle(LEDConstants.CAN_ID);

        var config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = LEDConstants.BRIGHTNESS;
        candle.getConfigurator().apply(config);

        clearAllAnimations();
        setSolid(LEDConstants.OFF);
    }

    @Override
    public void periodic() {
        updateAllianceColor();

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

        // Only update LEDs on state change
        if (newState != lastState) {
            clearAllAnimations();
            applyPattern(newState);
            SmartDashboard.putString("LED/State", newState.toString());
            currentState = newState;
            lastState = newState;
        }
    }

    private void applyPattern(LEDState state) {
        int start = LEDConstants.FIRST_LED;
        int end = LEDConstants.LAST_LED;

        switch (state) {
            case VISION_ALIGNED  -> setSolid(LEDConstants.GREEN);
            case AUTO_AIM_ACTIVE -> setSolid(LEDConstants.YELLOW);
            case INTAKING        -> setSolid(LEDConstants.ORANGE);

            case AUTONOMOUS -> candle.setControl(
                new RainbowAnimation(start, end)
                    .withSlot(0)
                    .withBrightness(1.0)
                    .withFrameRate(LEDConstants.RAINBOW_SPEED));

            case DISABLED -> candle.setControl(
                new SingleFadeAnimation(start, end)
                    .withSlot(0)
                    .withColor(allianceColor)
                    .withFrameRate(LEDConstants.FADE_SPEED));

            case END_GAME -> candle.setControl(
                new StrobeAnimation(start, end)
                    .withSlot(0)
                    .withColor(LEDConstants.RED)
                    .withFrameRate(LEDConstants.STROBE_SPEED));

            case SHIFT_1 -> candle.setControl(
                new ColorFlowAnimation(start, end)
                    .withSlot(0)
                    .withColor(allianceColor)
                    .withDirection(AnimationDirectionValue.Forward)
                    .withFrameRate(LEDConstants.FLOW_SPEED));

            case SHIFT_2 -> candle.setControl(
                new RgbFadeAnimation(start, end)
                    .withSlot(0)
                    .withBrightness(1.0)
                    .withFrameRate(LEDConstants.RGB_FADE_SPEED));

            case SHIFT_3 -> candle.setControl(
                new LarsonAnimation(start, end)
                    .withSlot(0)
                    .withColor(LEDConstants.RED)
                    .withSize(5)
                    .withBounceMode(LarsonBounceValue.Center)
                    .withFrameRate(LEDConstants.LARSON_SPEED));

            case SHIFT_4 -> candle.setControl(
                new SingleFadeAnimation(start, end)
                    .withSlot(0)
                    .withColor(LEDConstants.RED)
                    .withFrameRate(LEDConstants.STROBE_SPEED));

            case IDLE -> candle.setControl(
                new FireAnimation(start, end)
                    .withSlot(0)
                    .withBrightness(1.0)
                    .withFrameRate(LEDConstants.FIRE_SPEED));
        }
    }

    private void setSolid(RGBWColor color) {
        candle.setControl(
            new SolidColor(LEDConstants.FIRST_LED, LEDConstants.LAST_LED)
                .withColor(color));
    }

    private void clearAllAnimations() {
        for (int i = 0; i < 8; i++) {
            candle.setControl(new EmptyAnimation(i));
        }
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

    private void updateAllianceColor() {
        Alliance current = DriverStation.getAlliance().orElse(null);
        if (current != lastAlliance) {
            allianceColor = (current == Alliance.Red) ? LEDConstants.RED : LEDConstants.BLUE;
            lastAlliance = current;

            // Force re-send if currently showing an alliance-dependent pattern
            if (currentState == LEDState.DISABLED || currentState == LEDState.SHIFT_1) {
                lastState = null;
            }
        }
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}
