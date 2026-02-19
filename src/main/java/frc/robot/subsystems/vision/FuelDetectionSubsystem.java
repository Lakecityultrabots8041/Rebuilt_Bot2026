package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Handles the limelight-climb camera in Hailo neural detector mode.
 * Detects fuel on the field and provides TX/TY/area for the DriveToFuel command.
 *
 * Pipeline 0 = AprilTag (reserved if camera is repositioned for climbing assist).
 * Pipeline 1 = Neural detector (Hailo trained model for fuel detection).
 *
 * See docs/HAILO_TRAINING.md for model training and deployment steps.
 * All threshold constants are in VisionConstants — tune them with the trained model active.
 */
public class FuelDetectionSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-climb";

    private final NetworkTable table;
    private final NetworkTableEntry tv, tx, ty, ta;

    // Per-loop cache — read once in periodic, used everywhere else
    private boolean cachedHasTarget = false;
    private double cachedTx = 0;
    private double cachedTy = 0;
    private double cachedArea = 0;

    private final boolean isSimulation;

    public FuelDetectionSubsystem() {
        table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        isSimulation = RobotBase.isSimulation();

        if (!isSimulation) {
            activateFuelDetector();
        }
    }

    /** Switch to Hailo neural detector pipeline for fuel detection. */
    public void activateFuelDetector() {
        table.getEntry("pipeline").setNumber(VisionConstants.FUEL_DETECTOR_PIPELINE);
    }

    /** Switch to AprilTag pipeline — use if camera is repositioned for climbing assist. */
    public void activateAprilTagPipeline() {
        table.getEntry("pipeline").setNumber(VisionConstants.CLIMBING_PIPELINE);
    }

    /** True if the neural detector has a valid fuel target with sufficient area. */
    public boolean hasTarget() {
        if (isSimulation) return false;
        return cachedHasTarget && cachedArea >= VisionConstants.FUEL_MIN_AREA;
    }

    /** Horizontal offset to target in degrees. Positive = target is right of center. */
    public double getTX() {
        return isSimulation ? 0 : cachedTx;
    }

    /** Vertical offset to target in degrees. */
    public double getTY() {
        return isSimulation ? 0 : cachedTy;
    }

    /** Target area as percentage of the camera image. Larger = closer. */
    public double getArea() {
        return isSimulation ? 0 : cachedArea;
    }

    /** True if fuel is close enough to intake based on area threshold. */
    public boolean isCloseEnoughToIntake() {
        return hasTarget() && cachedArea >= VisionConstants.FUEL_INTAKE_AREA_THRESHOLD;
    }

    @Override
    public void periodic() {
        if (!isSimulation) {
            cachedHasTarget = tv.getDouble(0) == 1;
            cachedTx        = tx.getDouble(0.0);
            cachedTy        = ty.getDouble(0.0);
            cachedArea      = ta.getDouble(0.0);
        }

        SmartDashboard.putBoolean("Fuel/Has Target", hasTarget());
        SmartDashboard.putNumber("Fuel/TX (deg)", getTX());
        SmartDashboard.putNumber("Fuel/Area (%)", getArea());
        SmartDashboard.putBoolean("Fuel/Close Enough", isCloseEnoughToIntake());
        SmartDashboard.putBoolean("Fuel/Sim Mode", isSimulation);
    }
}
