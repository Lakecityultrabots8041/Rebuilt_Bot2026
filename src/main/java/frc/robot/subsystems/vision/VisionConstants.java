package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.Set;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Vision constants for 2026 REBUILT.
 * All distances in meters unless noted. See vision.md for field layout and tag details.
 */
public final class VisionConstants {

    private VisionConstants() {}

    // Target stop distances per tag — tune these on the real field

    public static final double[] APRILTAG_DISTANCES = new double[33]; // IDs 1-32, index 0 unused
    static {
        for (int i = 0; i < APRILTAG_DISTANCES.length; i++) {
            APRILTAG_DISTANCES[i] = 2.0;
        }

        // TODO: tune for our shooter
        double hubShootDistance = Units.inchesToMeters(97); // Measured from center of hub to front of Limelight lens at shooting position
        APRILTAG_DISTANCES[2]  = hubShootDistance;
        APRILTAG_DISTANCES[3]  = hubShootDistance;
        APRILTAG_DISTANCES[4]  = hubShootDistance;
        APRILTAG_DISTANCES[5]  = hubShootDistance;
        APRILTAG_DISTANCES[8]  = hubShootDistance;
        APRILTAG_DISTANCES[9]  = hubShootDistance;
        APRILTAG_DISTANCES[10] = hubShootDistance;
        APRILTAG_DISTANCES[11] = hubShootDistance;

        APRILTAG_DISTANCES[18] = hubShootDistance;
        APRILTAG_DISTANCES[19] = hubShootDistance;
        APRILTAG_DISTANCES[20] = hubShootDistance;
        APRILTAG_DISTANCES[21] = hubShootDistance;
        APRILTAG_DISTANCES[24] = hubShootDistance;
        APRILTAG_DISTANCES[25] = hubShootDistance;
        APRILTAG_DISTANCES[26] = hubShootDistance;
        APRILTAG_DISTANCES[27] = hubShootDistance;

        double towerApproachDistance = Units.inchesToMeters(36);
        APRILTAG_DISTANCES[15] = towerApproachDistance;
        APRILTAG_DISTANCES[16] = towerApproachDistance;

        APRILTAG_DISTANCES[31] = towerApproachDistance;
        APRILTAG_DISTANCES[32] = towerApproachDistance;

        double outpostDistance = Units.inchesToMeters(30);
        APRILTAG_DISTANCES[13] = outpostDistance;
        APRILTAG_DISTANCES[14] = outpostDistance;

        APRILTAG_DISTANCES[29] = outpostDistance;
        APRILTAG_DISTANCES[30] = outpostDistance;

        double trenchDistance = Units.inchesToMeters(48);
        APRILTAG_DISTANCES[1]  = trenchDistance;
        APRILTAG_DISTANCES[6]  = trenchDistance;
        APRILTAG_DISTANCES[7]  = trenchDistance;
        APRILTAG_DISTANCES[12] = trenchDistance;

        APRILTAG_DISTANCES[17] = trenchDistance;
        APRILTAG_DISTANCES[22] = trenchDistance;
        APRILTAG_DISTANCES[23] = trenchDistance;
        APRILTAG_DISTANCES[28] = trenchDistance;
    }

    public static final double DEFAULT_APRILTAG_DISTANCE = 2.0;

    // Tag groups — pass to Limelight_Move to filter which tags to align to
    public static final Set<Integer> BLUE_HUB_TAGS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    public static final Set<Integer> RED_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    public static final Set<Integer> ALL_HUB_TAGS = Set.of(
        2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

    public static final Set<Integer> BLUE_TOWER_TAGS = Set.of(15, 16);
    public static final Set<Integer> RED_TOWER_TAGS = Set.of(31, 32);
    public static final Set<Integer> ALL_TOWER_TAGS = Set.of(15, 16, 31, 32);

    public static final Set<Integer> BLUE_OUTPOST_TAGS = Set.of(13, 14);
    public static final Set<Integer> RED_OUTPOST_TAGS = Set.of(29, 30);
    public static final Set<Integer> ALL_OUTPOST_TAGS = Set.of(13, 14, 29, 30);

    public static final Set<Integer> BLUE_TRENCH_TAGS = Set.of(1, 6, 7, 12);
    public static final Set<Integer> RED_TRENCH_TAGS = Set.of(17, 22, 23, 28);
    public static final Set<Integer> ALL_TRENCH_TAGS = Set.of(1, 6, 7, 12, 17, 22, 23, 28);

    public static final Set<Integer> ALL_TAGS = Set.of(
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

    // Alliance-aware getters — resolved at runtime from DriverStation
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public static Set<Integer> getHubTags() {
        return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
    }

    public static Set<Integer> getTowerTags() {
        return isRedAlliance() ? RED_TOWER_TAGS : BLUE_TOWER_TAGS;
    }

    public static Set<Integer> getOutpostTags() {
        return isRedAlliance() ? RED_OUTPOST_TAGS : BLUE_OUTPOST_TAGS;
    }

    public static Set<Integer> getTrenchTags() {
        return isRedAlliance() ? RED_TRENCH_TAGS : BLUE_TRENCH_TAGS;
    }

    // Tag heights off floor
    public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(44.25);
    public static final double TOWER_OUTPOST_TAG_HEIGHT_METERS = Units.inchesToMeters(21.75);
    public static final double TRENCH_TAG_HEIGHT_METERS = Units.inchesToMeters(35.0);

    // Camera mounting — TODO: measure these on the actual robot
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(25.125);
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 0;

    // Alignment tolerances
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;
    public static final double DISTANCE_TOLERANCE_METERS = Units.inchesToMeters(3.0);
    public static final double STRAFE_TOLERANCE_METERS = Units.inchesToMeters(3.0);

    // Proportional gains
    public static final double ROTATION_GAIN = 0.03;
    public static final double FORWARD_GAIN = 0.8;
    public static final double AUTO_STRAFE_GAIN = 0.5;

    // Speed limits (fraction of max, 0.0–1.0)
    public static final double MAX_ROTATION_SPEED = 0.25;
    public static final double MAX_FORWARD_SPEED = 0.30;
    public static final double MAX_DRIVER_STRAFE_SCALE = 0.5;
    public static final double MAX_AUTO_STRAFE_SPEED = 0.20;

    // Safety
    public static final double MIN_TARGET_AREA = 0.1;
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 8.0;
    public static final int ALIGNED_LOOPS_REQUIRED = 25; // ~0.5s at 50Hz

    // Auto-aim
    public static final double AUTO_AIM_MAX_RANGE_METERS = 5.0;
    public static final double AUTO_AIM_KP = 4.0;
    public static final double AUTO_AIM_KI = 0.0;
    public static final double AUTO_AIM_KD = 0.1;
    public static final double AUTO_AIM_MAX_ROTATION_RATE = 1.5;  // rad/s
    public static final double AUTO_AIM_SLEW_RATE = 3.0;          // rad/s²

    public static double getAprilTagDistance(int tagID) {
        if (tagID >= 0 && tagID < APRILTAG_DISTANCES.length) {
            return APRILTAG_DISTANCES[tagID];
        }
        return DEFAULT_APRILTAG_DISTANCE;
    }

}