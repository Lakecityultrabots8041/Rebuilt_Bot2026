package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
    private LEDConstants() {}

    // ===== Hardware =====
    // PWM port on the roboRIO where the LED data line is plugged in.
    // Update this when the strip is wired.
    public static final int PWM_PORT = 0;

    // Total number of LEDs across all chained strips.
    // Update this when strip length is known.
    public static final int LED_COUNT = 60;

    // ===== Pattern Timing =====
    public static final double SHOOTING_BLINK_SECONDS = 0.1;
    public static final double AUTO_AIM_BREATHE_SECONDS = 1.0;
    public static final double DISABLED_BREATHE_SECONDS = 2.0;
    public static final double IDLE_BRIGHTNESS_PERCENT = 25.0;
    public static final double RAINBOW_SCROLL_HZ = 2.0;

    // ===== Colors =====
    public static final Color SHOOTING_COLOR   = Color.kWhite;
    public static final Color ALIGNED_COLOR    = Color.kGreen;
    public static final Color AUTO_AIM_COLOR   = Color.kYellow;
    public static final Color INTAKING_COLOR   = Color.kOrangeRed;
    public static final Color BLUE_ALLIANCE    = Color.kBlue;
    public static final Color RED_ALLIANCE     = Color.kRed;
    public static final Color DEFAULT_ALLIANCE = Color.kBlue;
}
