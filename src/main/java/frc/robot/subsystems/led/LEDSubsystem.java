package frc.robot.subsystems.led;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    // Conditions checked every loop to decide what pattern to show.
    // These are passed in from RobotContainer so LED logic stays in one file.
    private final BooleanSupplier isShootingSupplier;
    private final BooleanSupplier visionAlignedSupplier;
    private final BooleanSupplier autoAimActiveSupplier;
    private final BooleanSupplier intakingSupplier;

    // Patterns built once and reused every loop
    private final LEDPattern shootingPattern;
    private final LEDPattern visionAlignedPattern;
    private final LEDPattern autoAimActivePattern;
    private final LEDPattern intakingPattern;
    private final LEDPattern autonomousPattern;

    // Alliance patterns rebuild when FMS connects and alliance is known
    private LEDPattern disabledPattern;
    private LEDPattern idlePattern;
    private Alliance lastAlliance = null;

    private LEDState currentState = LEDState.IDLE;

    // Highest priority at the top, lowest at the bottom.
    // The first condition that is true wins.
    public enum LEDState {
        SHOOTING,         // Flywheel at speed AND feed running
        VISION_ALIGNED,   // Auto-aim on AND locked on hub tag
        AUTO_AIM_ACTIVE,  // Auto-aim toggle on, searching
        INTAKING,         // Intake rollers running
        AUTONOMOUS,       // Running an auto routine
        DISABLED,         // Robot disabled
        IDLE              // Nothing happening
    }

    public LEDSubsystem(
            BooleanSupplier isShootingSupplier,
            BooleanSupplier visionAlignedSupplier,
            BooleanSupplier autoAimActiveSupplier,
            BooleanSupplier intakingSupplier) {

        this.isShootingSupplier    = isShootingSupplier;
        this.visionAlignedSupplier = visionAlignedSupplier;
        this.autoAimActiveSupplier = autoAimActiveSupplier;
        this.intakingSupplier      = intakingSupplier;

        // Hardware setup. Only one AddressableLED per roboRIO.
        led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        // Build patterns once
        shootingPattern = LEDPattern.solid(LEDConstants.SHOOTING_COLOR)
            .blink(Seconds.of(LEDConstants.SHOOTING_BLINK_SECONDS));
        visionAlignedPattern = LEDPattern.solid(LEDConstants.ALIGNED_COLOR);
        autoAimActivePattern = LEDPattern.solid(LEDConstants.AUTO_AIM_COLOR)
            .breathe(Seconds.of(LEDConstants.AUTO_AIM_BREATHE_SECONDS));
        intakingPattern = LEDPattern.solid(LEDConstants.INTAKING_COLOR);
        autonomousPattern = LEDPattern.rainbow(255, 128)
            .scrollAtRelativeSpeed(Hertz.of(LEDConstants.RAINBOW_SCROLL_HZ));

        // Default until FMS tells us which alliance we are
        buildAlliancePatterns(LEDConstants.DEFAULT_ALLIANCE);
    }

    @Override
    public void periodic() {
        updateAlliancePatterns();

        // Priority waterfall: first match wins
        LEDState newState;
        if (isShootingSupplier.getAsBoolean()) {
            newState = LEDState.SHOOTING;
        } else if (visionAlignedSupplier.getAsBoolean()) {
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

        // Apply pattern and push to hardware
        getPattern(newState).applyTo(ledBuffer);
        led.setData(ledBuffer);

        // Dashboard (only log on change)
        if (newState != currentState) {
            SmartDashboard.putString("LED/State", newState.toString());
            currentState = newState;
        }
    }

    private LEDPattern getPattern(LEDState state) {
        return switch (state) {
            case SHOOTING        -> shootingPattern;
            case VISION_ALIGNED  -> visionAlignedPattern;
            case AUTO_AIM_ACTIVE -> autoAimActivePattern;
            case INTAKING        -> intakingPattern;
            case AUTONOMOUS      -> autonomousPattern;
            case DISABLED        -> disabledPattern;
            case IDLE            -> idlePattern;
        };
    }

    // Rebuild alliance patterns when FMS connects and tells us red or blue
    private void updateAlliancePatterns() {
        Alliance current = DriverStation.getAlliance().orElse(null);
        if (current != lastAlliance) {
            Color allianceColor = (current == Alliance.Red)
                ? LEDConstants.RED_ALLIANCE
                : LEDConstants.BLUE_ALLIANCE;
            buildAlliancePatterns(allianceColor);
            lastAlliance = current;
        }
    }

    private void buildAlliancePatterns(Color allianceColor) {
        disabledPattern = LEDPattern.solid(allianceColor)
            .breathe(Seconds.of(LEDConstants.DISABLED_BREATHE_SECONDS));
        idlePattern = LEDPattern.solid(allianceColor)
            .atBrightness(Percent.of(LEDConstants.IDLE_BRIGHTNESS_PERCENT));
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}
