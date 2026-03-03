package frc.robot.subsystems.led;

// See docs/LED_GUIDE.md for patterns and how to change them.
public final class LEDConstants {
    private LEDConstants() {}

    public static final int PWM_PORT = 0;

    // ===== Action Patterns =====
    public static final double VISION_ALIGNED  =  0.75;  // Solid Green
    public static final double AUTO_AIM_ACTIVE =  0.67;  // Solid Yellow
    public static final double INTAKING        =  0.63;  // Solid Orange
    public static final double AUTONOMOUS      = -0.99;  // Rainbow

    // ===== Alliance Patterns =====
    public static final double DISABLED_RED    = -0.17;  // Breath Red
    public static final double DISABLED_BLUE   = -0.15;  // Breath Blue
    public static final double DEFAULT_DISABLED = -0.15; // Fallback before FMS connects

    // ===== Teleop Shift Patterns =====
    public static final double SHIFT_1   = -0.45;  // Color Waves, Rainbow Palette
    public static final double SHIFT_2   = -0.69;  // Beats Per Minute, Rainbow Palette
    public static final double SHIFT_3   = -0.35;  // Larson Scanner, Red
    public static final double SHIFT_4   = -0.25;  // Heartbeat, Red
    public static final double END_GAME  = -0.11;  // Strobe, Red

    public static final double IDLE      = -0.57;  // Fire, Large
}
