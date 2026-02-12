package frc.robot.subsystems.vision;

import java.util.Optional;
import java.util.Set;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Constants for the vision/Limelight subsystem.
 * 2026 REBUILT presented by Haas — ALL DATA FROM OFFICIAL GAME MANUAL.
 * 
 * ALL DISTANCES ARE IN METERS unless explicitly noted otherwise.
 * 
 * =====================================================================
 * 2026 REBUILT AprilTag Layout (32 tags, 36h11 family, IDs 1-32)
 * =====================================================================
 * 
 * HUB TAGS (16 total, 2 per face × 4 faces per HUB):
 *   Blue HUB:  2, 3, 4, 5, 8, 9, 10, 11
 *   Red HUB:   18, 19, 20, 21, 24, 25, 26, 27
 *   Height: centers at 44.25in (1.124m) off floor
 * 
 * TOWER WALL TAGS (4 total, 2 per tower):
 *   Blue Tower: 15, 16
 *   Red Tower:  31, 32
 *   Height: centers at 21.75in (0.5525m) off floor
 * 
 * OUTPOST TAGS (4 total, 2 per outpost):
 *   Blue Outpost: 13, 14
 *   Red Outpost:  29, 30
 *   Height: centers at 21.75in (0.5525m) off floor
 * 
 * TRENCH TAGS (8 total, 2 per trench, 1 facing alliance zone, 1 facing neutral zone):
 *   Blue Trenches: 1, 6, 7, 12
 *   Red Trenches:  17, 22, 23, 28
 *   Height: centers at 35in (0.889m) off floor
 * 
 * =====================================================================
 * Key Field Dimensions (from manual)
 * =====================================================================
 *   Field: 317.7in × 651.2in (~8.07m × 16.54m)
 *   HUB: 47in × 47in (1.19m × 1.19m), centered 158.6in (4.03m) from alliance wall
 *   HUB opening front edge: 72in (1.83m) off carpet
 *   TOWER: 49.25in wide × 45in deep × 78.25in tall
 *     LOW RUNG:  27.0in (0.686m) from floor
 *     MID RUNG:  45.0in (1.143m) from floor
 *     HIGH RUNG: 63.0in (1.600m) from floor
 *   TRENCH clearance: 50.34in wide × 22.25in tall (robot must fit under)
 *   BUMP: 6.513in (16.54cm) tall, 15-degree ramps
 *   DEPOT: 42in wide × 27in deep, walls ~1.125in tall
 *   OUTPOST opening: 31.8in wide × 7in tall, bottom at 28.1in off floor
 */
public final class VisionConstants {

    private VisionConstants() {}

    // =====================================================================
    // APRILTAG TARGET DISTANCES (METERS)
    // How far the robot should stop from each tag when auto-aligning.
    // We will need to tune these distances for final shooting accuracy and climbing engagement
    // =====================================================================
    public static final double[] APRILTAG_DISTANCES = new double[33]; // IDs 1-32, index 0 unused
    static {
        // Default for any unknown tag
        for (int i = 0; i < APRILTAG_DISTANCES.length; i++) {
            APRILTAG_DISTANCES[i] = 2.0; // 2 meters default, 
        }

        // --- BLUE HUB TAGS (IDs 2, 3, 4, 5, 8, 9, 10, 11) ---
        // HUB opening is 72in (1.83m) off carpet. Robot needs to shoot up into it.

        // TODO: Tune this distance based on our shooter's ideal range!
        double hubShootDistance = Units.inchesToMeters(97); // ~2.46m from tag face
        APRILTAG_DISTANCES[2]  = hubShootDistance;
        APRILTAG_DISTANCES[3]  = hubShootDistance;
        APRILTAG_DISTANCES[4]  = hubShootDistance;
        APRILTAG_DISTANCES[5]  = hubShootDistance;
        APRILTAG_DISTANCES[8]  = hubShootDistance;
        APRILTAG_DISTANCES[9]  = hubShootDistance;
        APRILTAG_DISTANCES[10] = hubShootDistance;
        APRILTAG_DISTANCES[11] = hubShootDistance;

        // --- RED HUB TAGS (IDs 18, 19, 20, 21, 24, 25, 26, 27) ---
        APRILTAG_DISTANCES[18] = hubShootDistance;
        APRILTAG_DISTANCES[19] = hubShootDistance;
        APRILTAG_DISTANCES[20] = hubShootDistance;
        APRILTAG_DISTANCES[21] = hubShootDistance;
        APRILTAG_DISTANCES[24] = hubShootDistance;
        APRILTAG_DISTANCES[25] = hubShootDistance;
        APRILTAG_DISTANCES[26] = hubShootDistance;
        APRILTAG_DISTANCES[27] = hubShootDistance;

        // --- BLUE TOWER WALL TAGS (IDs 15, 16) ---
        // Tower approach for climbing. Get close to engage rungs.
        double towerApproachDistance = Units.inchesToMeters(36); // ~0.91m
        APRILTAG_DISTANCES[15] = towerApproachDistance;
        APRILTAG_DISTANCES[16] = towerApproachDistance;

        // --- RED TOWER WALL TAGS (IDs 31, 32) ---
        APRILTAG_DISTANCES[31] = towerApproachDistance;
        APRILTAG_DISTANCES[32] = towerApproachDistance;

        // --- BLUE OUTPOST TAGS (IDs 13, 14) ---
        // Outpost approach for fuel pickup/delivery from human player
        double outpostDistance = Units.inchesToMeters(30); // ~0.76m close for handoff
        APRILTAG_DISTANCES[13] = outpostDistance;
        APRILTAG_DISTANCES[14] = outpostDistance;

        // --- RED OUTPOST TAGS (IDs 29, 30) ---
        APRILTAG_DISTANCES[29] = outpostDistance;
        APRILTAG_DISTANCES[30] = outpostDistance;

        // --- BLUE TRENCH TAGS (IDs 1, 6, 7, 12) ---
        // Trench drive-through alignment. Center under the trench arm.
        double trenchDistance = Units.inchesToMeters(48); // ~1.22m
        APRILTAG_DISTANCES[1]  = trenchDistance;
        APRILTAG_DISTANCES[6]  = trenchDistance;
        APRILTAG_DISTANCES[7]  = trenchDistance;
        APRILTAG_DISTANCES[12] = trenchDistance;

        // --- RED TRENCH TAGS (IDs 17, 22, 23, 28) ---
        APRILTAG_DISTANCES[17] = trenchDistance;
        APRILTAG_DISTANCES[22] = trenchDistance;
        APRILTAG_DISTANCES[23] = trenchDistance;
        APRILTAG_DISTANCES[28] = trenchDistance;
    }

    /** Default distance if unknown tag (meters) */
    public static final double DEFAULT_APRILTAG_DISTANCE = 2.0;

    // =====================================================================
    // TAG GROUPS — use these to tell Limelight_Move which tags to accept
    // =====================================================================

    /** All Blue HUB tags (for scoring fuel) */
    public static final Set<Integer> BLUE_HUB_TAGS = Set.of(2, 3, 4, 5, 8, 9, 10, 11);

    /** All Red HUB tags (for scoring fuel) */
    public static final Set<Integer> RED_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

    /** ALL HUB tags (both alliances — use if you don't care which hub) */
    public static final Set<Integer> ALL_HUB_TAGS = Set.of(
        2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

    /** Blue TOWER tags (for climbing) */
    public static final Set<Integer> BLUE_TOWER_TAGS = Set.of(15, 16);

    /** Red TOWER tags (for climbing) */
    public static final Set<Integer> RED_TOWER_TAGS = Set.of(31, 32);

    /** ALL TOWER tags */
    public static final Set<Integer> ALL_TOWER_TAGS = Set.of(15, 16, 31, 32);

    /** Blue OUTPOST tags (human player fuel handoff) */
    public static final Set<Integer> BLUE_OUTPOST_TAGS = Set.of(13, 14);

    /** Red OUTPOST tags */
    public static final Set<Integer> RED_OUTPOST_TAGS = Set.of(29, 30);

    /** ALL OUTPOST tags */
    public static final Set<Integer> ALL_OUTPOST_TAGS = Set.of(13, 14, 29, 30);

    /** Blue TRENCH tags */
    public static final Set<Integer> BLUE_TRENCH_TAGS = Set.of(1, 6, 7, 12);

    /** Red TRENCH tags */
    public static final Set<Integer> RED_TRENCH_TAGS = Set.of(17, 22, 23, 28);

    /** ALL TRENCH tags */
    public static final Set<Integer> ALL_TRENCH_TAGS = Set.of(1, 6, 7, 12, 17, 22, 23, 28);

    /** Every tag on the field */
    public static final Set<Integer> ALL_TAGS = Set.of(
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32);

    // =====================================================================
    // ALLIANCE-AWARE TAG GETTERS
    // Reads from DriverStation automatically. Falls back to Blue if
    // alliance isn't set yet (e.g. in sim before connecting to FMS).
    // =====================================================================

    /** Returns true if we're on the Red alliance */
    public static boolean isRedAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    /** HUB tags for YOUR alliance (scoring) */
    public static Set<Integer> getHubTags() {
        return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
    }

    /** TOWER tags for YOUR alliance (climbing) */
    public static Set<Integer> getTowerTags() {
        return isRedAlliance() ? RED_TOWER_TAGS : BLUE_TOWER_TAGS;
    }

    /** OUTPOST tags for YOUR alliance (human player handoff) */
    public static Set<Integer> getOutpostTags() {
        return isRedAlliance() ? RED_OUTPOST_TAGS : BLUE_OUTPOST_TAGS;
    }

    /** TRENCH tags for YOUR alliance */
    public static Set<Integer> getTrenchTags() {
        return isRedAlliance() ? RED_TRENCH_TAGS : BLUE_TRENCH_TAGS;
    }

    // =====================================================================
    // TAG HEIGHT REFERENCE DISTANCES (METERS)
    // =====================================================================
    /** HUB tag centers: 44.25in off floor */
    public static final double HUB_TAG_HEIGHT_METERS = Units.inchesToMeters(44.25);

    /** Tower/Outpost tag centers: 21.75in off floor */
    public static final double TOWER_OUTPOST_TAG_HEIGHT_METERS = Units.inchesToMeters(21.75);

    /** Trench tag centers: 35in off floor */
    public static final double TRENCH_TAG_HEIGHT_METERS = Units.inchesToMeters(35.0);

    // =====================================================================
    // CAMERA MOUNTING (METERS)
    // =====================================================================
    /** Height of the Limelight lens from the floor */
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12); // TODO: measure!

    /** Camera mount angle (0 = horizontal, positive = tilted up) */
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 45; // TODO: measure!

    // =====================================================================
    // ALIGNMENT TOLERANCES
    // =====================================================================
    /** How many degrees of horizontal offset is "aligned" */
    public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;

    /** How many meters of distance error is "at distance" */
    public static final double DISTANCE_TOLERANCE_METERS = Units.inchesToMeters(3.0); // 3in = 0.076m

    /** How many meters of lateral offset is "centered" (for auto-strafe) */
    public static final double STRAFE_TOLERANCE_METERS = Units.inchesToMeters(3.0); // 3in = 0.076m

    // =====================================================================
    // PROPORTIONAL GAINS
    // =====================================================================
    /** Rotation gain: degrees of tx error → fraction of max rotation speed */
    public static final double ROTATION_GAIN = 0.03;

    /** Forward gain: meters of distance error → fraction of max forward speed */
    public static final double FORWARD_GAIN = 0.8;

    /** Auto-strafe gain: meters of lateral offset → fraction of max strafe speed.
     *  Used in autonomous to correct lateral drift (example: after a bump). */
    public static final double AUTO_STRAFE_GAIN = 0.5;

    // =====================================================================
    // SPEED LIMITS (fraction of max speed, 0.0 to 1.0)
    // =====================================================================
    public static final double MAX_ROTATION_SPEED = 0.25;
    public static final double MAX_FORWARD_SPEED = 0.30;
    public static final double MAX_DRIVER_STRAFE_SCALE = 0.5;

    /** Max auto-strafe speed in autonomous — conservative since strafe is
     *  only needed for small corrections, not primary movement. */
    public static final double MAX_AUTO_STRAFE_SPEED = 0.20;

    // =====================================================================
    // SAFETY
    // =====================================================================
    /** Minimum target area to consider valid (prevents false positives from far away) */
    public static final double MIN_TARGET_AREA = 0.1;

    /** Max time for alignment command before timeout (seconds) */
    public static final double ALIGNMENT_TIMEOUT_SECONDS = 8.0;

    /** Consecutive aligned loops needed to finish (1 loop ≈ 20ms, 25 loops ≈ 0.5s) */
    public static final int ALIGNED_LOOPS_REQUIRED = 25;

    // =====================================================================
    // AUTO-AIM CONSTANTS
    // =====================================================================
    /** Maximum range to track a hub tag for auto-aim (meters) */
    public static final double AUTO_AIM_MAX_RANGE_METERS = 5.0;

    /** Auto-aim PID proportional gain — gentle to avoid oscillation */
    public static final double AUTO_AIM_KP = 4.0;
    /** Auto-aim PID integral gain */
    public static final double AUTO_AIM_KI = 0.0;
    /** Auto-aim PID derivative gain — small D for damping */
    public static final double AUTO_AIM_KD = 0.1;

    /** Max rotation rate for auto-aim (rad/s) — prevents fast snapping */
    public static final double AUTO_AIM_MAX_ROTATION_RATE = 1.5;
    /** Slew rate limit for auto-aim rotation (rad/s²) — smooths acceleration */
    public static final double AUTO_AIM_SLEW_RATE = 3.0;

    // =====================================================================
    // HELPER
    // =====================================================================
    public static double getAprilTagDistance(int tagID) {
        if (tagID >= 0 && tagID < APRILTAG_DISTANCES.length) {
            return APRILTAG_DISTANCES[tagID];
        }
        return DEFAULT_APRILTAG_DISTANCE;
    }
}