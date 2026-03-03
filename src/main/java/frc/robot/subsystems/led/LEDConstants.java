package frc.robot.subsystems.led;

public final class LEDConstants {
    private LEDConstants() {}

    // ===== Hardware =====
    // PWM port on the roboRIO where the Blinkin signal wire is plugged in.
    public static final int PWM_PORT = 0;

    // ===== Blinkin PWM Pattern Values =====
    // Each value selects a built-in pattern on the Rev Blinkin.
    // See the printed pattern table that ships with the Blinkin for the full list.
    public static final double VISION_ALIGNED  =  0.75;  // Solid Green
    public static final double AUTO_AIM_ACTIVE =  0.67;  // Solid Yellow
    public static final double INTAKING        =  0.63;  // Solid Orange
    public static final double AUTONOMOUS      = -0.99;  // Rainbow

    // ===== Alliance Patterns =====
    public static final double DISABLED_RED    = -0.17;  // Breath Red
    public static final double DISABLED_BLUE   = -0.15;  // Breath Blue
    public static final double IDLE_RED        =  0.59;  // Solid Red (actually Hot Pink on table, see note)
    public static final double IDLE_BLUE       =  0.85;  // Solid Blue
    public static final double DEFAULT_IDLE    =  0.85;  // Solid Blue (fallback before FMS connects)
    public static final double DEFAULT_DISABLED = -0.15; // Breath Blue (fallback before FMS connects)
}
