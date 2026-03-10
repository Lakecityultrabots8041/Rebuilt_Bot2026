package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

// See docs/LED_GUIDE.md for patterns and how to change them.
public final class LEDConstants {
    private LEDConstants() {}

    // CANdle on default CAN bus, CAN ID 1
    public static final int CAN_ID = 1;

    // 8 onboard LEDs (indices 0-7) + 25 external strip LEDs (indices 8-32)
    public static final int ONBOARD_LEDS = 8;
    public static final int STRIP_LENGTH = 25;
    public static final int TOTAL_LEDS = ONBOARD_LEDS + STRIP_LENGTH;
    public static final int FIRST_LED = 0;
    public static final int LAST_LED = TOTAL_LEDS - 1;

    // Global brightness scalar (0.0 to 1.0)
    public static final double BRIGHTNESS = 0.75;

    // ===== Colors =====
    public static final RGBWColor GREEN  = new RGBWColor(0, 255, 0);
    public static final RGBWColor YELLOW = new RGBWColor(255, 255, 0);
    public static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
    public static final RGBWColor RED    = new RGBWColor(255, 0, 0);
    public static final RGBWColor BLUE   = new RGBWColor(0, 0, 255);
    public static final RGBWColor OFF    = new RGBWColor(0, 0, 0);

    // ===== Animation Frame Rates (Hz) =====
    public static final double RAINBOW_SPEED  = 25.0;
    public static final double FIRE_SPEED     = 25.0;
    public static final double STROBE_SPEED   = 10.0;
    public static final double FADE_SPEED     = 5.0;
    public static final double FLOW_SPEED     = 15.0;
    public static final double LARSON_SPEED   = 15.0;
    public static final double RGB_FADE_SPEED = 10.0;
}
