package frc.robot.subsystems.drive;

/** Drive subsystem constants. See docs/DRIVE_TUNING.md for tuning process and timing reference. */
public final class DriveConstants {

    private DriveConstants() {}

    // Asymmetric slew: accel and decel are limited separately.
    // Tune ACCEL freely. Tune DECEL carefully — too high tips the robot on hard stops.
    public static final double MAX_TELEOP_ACCEL = 8.0;
    public static final double MAX_TELEOP_DECEL = 4.0;


    
    // Applied in CommandSwerveDrivetrain, not TunerConstants — survives Tuner X regeneration.
    // See docs/DRIVE_TUNING.md for tuning guidance.
    public static final double DRIVE_STATOR_CURRENT_LIMIT_AMPS = 80.0;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = 60.0;
}
