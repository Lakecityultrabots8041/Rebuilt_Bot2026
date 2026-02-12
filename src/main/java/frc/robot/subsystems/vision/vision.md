# FRC Vision System Documentation

**Author:** Chris - Team 8041 Lake City Ultrabots  
**Last Updated:** February 2026  
**Target Audience:** FRC teams using Limelight 4 with Phoenix 6 swerve drivetrains

## System Overview

This vision system provides autonomous alignment to AprilTags on the 2026 REBUILT field. The architecture supports:

- Multi-tag alignment (any tag in a group, not just one specific ID)
- Alliance-aware tag selection (automatically picks red/blue tags based on DriverStation)
- Teleop with driver strafe input
- Full autonomous operation
- Simulation support for testing without hardware
- Vision-odometry fusion via MegaTag2

**Hardware Requirements:**
- Limelight 4 (running latest firmware)
- CTRE Phoenix 6 swerve drivetrain
- RoboRIO 2.0

## Architecture

### Component Breakdown

**LimelightSubsystem.java** - Hardware interface and data processing
- Reads NetworkTables data from Limelight
- Calculates distances using MegaTag2 camera-space coordinates
- Simulates vision in robot simulator using field layout
- Feeds vision measurements to drivetrain odometry

**VisionConstants.java** - Configuration and field data
- AprilTag groups (hub, tower, outpost, trench)
- Target distances for each tag type
- PID gains and speed limits
- Alliance-aware helper methods

**Limelight_Move.java** - Alignment command
- Three-axis control: rotation, forward/back, strafe
- Works with tag groups instead of single tags
- Supports both teleop and autonomous modes

**RobotContainer.java** - Command factories and button bindings
- Creates command instances with alliance-aware tag suppliers
- Wires up auto-aim for driver control
- Registers PathPlanner named commands

## How Distance Calculation Works

The key to this system is getting accurate distance to the AprilTag. We use MegaTag2's camera-space coordinates instead of calculating distance from pixel height.

### What is Camera-Space?

When Limelight sees an AprilTag, it provides the tag's position relative to the camera in three dimensions:

- **X axis:** Lateral offset (positive = tag is to the camera's right)
- **Y axis:** Vertical offset (positive = tag is above camera center)
- **Z axis:** Forward distance (positive = tag is in front of camera)

This is in meters, already calculated by Limelight's pose estimation.

### Code Implementation

```java
// From LimelightSubsystem.java - updateMegaTag2Cache()
double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);

if (targetPose != null && targetPose.length >= 3 && tv.getDouble(0) == 1) {
    cachedLateralOffsetMeters = targetPose[0]; // X = lateral offset
    cachedDistanceMeters = targetPose[2];      // Z = forward distance
    megatag2Available = true;
}
```

**Why this matters:** Traditional methods calculate distance from the pixel height of the tag using trigonometry. That requires knowing the exact tag height and camera mount angle. Camera-space data comes pre-calculated from Limelight's 3D pose solver, which uses the entire tag's pose in 3D space. It's more accurate and requires less tuning.

### Using the Distance Data

```java
// Get distance in meters (internal calculations)
double distance = limelight.getDistanceMeters();

// Get distance in inches (display only)
double distanceInches = limelight.getDistanceInchesForDisplay();
```

**Important:** All internal calculations use meters. Only convert to inches for SmartDashboard display. This prevents unit conversion bugs.

## Alliance-Aware Tag Selection

The 2026 REBUILT field has symmetric AprilTags for red and blue alliance. Your robot needs to align to YOUR alliance's tags, not the opponent's.

### How It Works

VisionConstants provides Supplier methods that read the alliance from DriverStation:

```java
// From VisionConstants.java
public static Set<Integer> getHubTags() {
    return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
}

public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == Alliance.Red;
}
```

This gets called when the command starts, not when you create the command object. This is critical for autonomous where you don't know your alliance until the match starts.

### Command Factory Pattern

```java
// From RobotContainer.java
private Limelight_Move createHubAlign() {
    return new Limelight_Move(drivetrain, limelight,
        VisionConstants::getHubTags,  // Method reference - evaluated at command start
        () -> -controller.getLeftX());
}
```

**Why method references:** The `VisionConstants::getHubTags` syntax passes the METHOD itself, not the result. Each time the command starts, it calls the method and gets the current alliance's tags.

**Wrong way:**
```java
// DON'T DO THIS - evaluates once at robot boot
new Limelight_Move(drivetrain, limelight,
    () -> VisionConstants.BLUE_HUB_TAGS, // Hardcoded - wrong!
    strafeSupplier);
```

## Three-Axis Control System

Limelight_Move controls three axes simultaneously:

### 1. Rotation (Omega)

Centers the robot on the tag horizontally using TX (horizontal offset in degrees).

```java
// Positive TX = target is to the right
// We need to rotate left (negative omega) to center it
double rotationOutput = -horizontalErrorDeg * VisionConstants.ROTATION_GAIN;
rotationOutput = MathUtil.clamp(rotationOutput,
    -VisionConstants.MAX_ROTATION_SPEED, VisionConstants.MAX_ROTATION_SPEED);
double rotationVelocityRps = rotationOutput * maxAngularRateRps;
```

**Gain tuning:** Start with ROTATION_GAIN = 0.03. Increase if rotation is too slow. Decrease if it oscillates.

### 2. Forward/Backward (X velocity)

Drives to the target distance using camera-space Z coordinate.

```java
// Positive error = too far away, drive forward
double distanceErrorMeters = currentDistanceMeters - targetDistanceMeters;
double forwardOutput = distanceErrorMeters * VisionConstants.FORWARD_GAIN;
forwardOutput = MathUtil.clamp(forwardOutput,
    -VisionConstants.MAX_FORWARD_SPEED, VisionConstants.MAX_FORWARD_SPEED);
double forwardVelocityMps = forwardOutput * maxSpeedMps;
```

**Gain tuning:** Start with FORWARD_GAIN = 0.8. This is higher than rotation because distance errors are usually larger than angle errors.

### 3. Strafe (Y velocity)

Two modes depending on context:

**Teleop mode (driver input present):**
```java
if (Math.abs(driverStrafe) > 0.0) {
    // Driver controls strafe - allows arcing approaches
    strafeVelocityMps = driverStrafe * maxSpeedMps * MAX_DRIVER_STRAFE_SCALE;
}
```

**Autonomous mode (no driver input):**
```java
else {
    // Auto-correct lateral offset using camera-space X coordinate
    double strafeOutput = lateralOffsetMeters * VisionConstants.AUTO_STRAFE_GAIN;
    strafeOutput = MathUtil.clamp(strafeOutput,
        -VisionConstants.MAX_AUTO_STRAFE_SPEED, VisionConstants.MAX_AUTO_STRAFE_SPEED);
    strafeVelocityMps = strafeOutput * maxSpeedMps;
}
```

This dual behavior lets drivers manually strafe during alignment (useful for avoiding defense) while still getting perfect centering in autonomous.

## Auto-Aim System

The auto-aim feature locks the robot's heading onto a hub tag while the driver maintains full translational control.

### How It Works

Instead of a separate command, auto-aim modifies the drivetrain's default command. When enabled:

1. Driver controls X/Y velocity normally
2. System overrides rotation with PID control
3. Shooter RPM adjusts based on distance

### Implementation

```java
// From RobotContainer.java - drivetrain default command
if (autoAimEnabled && limelight.isTrackingHubTag()) {
    // Calculate target heading from TX offset
    double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
    double targetHeading = currentHeading - Math.toRadians(limelight.getHorizontalOffset());

    // PID to target heading
    double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
    double clampedOutput = MathUtil.clamp(rawOutput,
        -VisionConstants.AUTO_AIM_MAX_ROTATION_RATE,
         VisionConstants.AUTO_AIM_MAX_ROTATION_RATE);
    double smoothOutput = autoAimSlew.calculate(clampedOutput);

    // Update shooter based on distance
    double distance = limelight.getDistanceMeters();
    if (distance > 0) {
        shooterSubsystem.setVariableVelocity(
            ShooterConstants.getVelocityForDistance(distance));
    }

    return drive.withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(smoothOutput);
}
```

### PID Configuration

The auto-aim PID uses continuous input since heading wraps around at ±180 degrees:

```java
autoAimPID.enableContinuousInput(-Math.PI, Math.PI);
```

This prevents the PID from trying to rotate 359 degrees when it only needs to rotate 1 degree in the opposite direction.

**Slew rate limiter** smooths acceleration to prevent jerky movements:

```java
private final SlewRateLimiter autoAimSlew = 
    new SlewRateLimiter(VisionConstants.AUTO_AIM_SLEW_RATE);
```

### Range Limiting

Auto-aim only activates when:
- A hub tag is visible
- Distance is positive (valid MegaTag2 data)
- Distance is under AUTO_AIM_MAX_RANGE_METERS (5.0m default)

This prevents the system from trying to shoot from across the field.

### State Management

When auto-aim deactivates, the system resets to prevent stale state:

```java
else {
    autoAimSlew.reset(0);      // Reset slew limiter
    autoAimPID.reset();        // Reset PID accumulators
    
    // Return shooter to idle if it was in variable mode
    if (shooterSubsystem.getState() == ShooterSubsystem.ShooterState.VARIABLE) {
        shooterSubsystem.setVariableVelocity(0);
    }
}
```

## Simulation Support

The system includes full simulation support for testing without hardware.

### How Simulation Works

When `RobotBase.isSimulation()` is true, LimelightSubsystem switches to simulation mode:

1. Gets the robot's simulated pose from the drivetrain
2. Loads the official 2026 field AprilTag layout
3. Calculates which tags would be visible based on:
   - Camera FOV (29.8 degrees horizontal)
   - Maximum detection range (8.0 meters)
   - Robot heading and tag positions
4. Generates fake TX, distance, and area values
5. Returns the closest visible tag

### Enabling Simulation

You must provide the robot pose supplier after creating the subsystem:

```java
// From RobotContainer.java
limelight.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
```

This gives the vision system access to the simulated robot position.

### Simulation Calculations

```java
// From LimelightSubsystem.java - updateSimVision()
for (var tag : fieldLayout.getTags()) {
    // Get tag position from field layout
    Pose3d tagPose3d = fieldLayout.getTagPose(tag.ID).get();
    Translation2d tagPos = tagPose3d.toPose2d().getTranslation();
    
    // Calculate distance
    double distance = robotPos.getDistance(tagPos);
    if (distance > MAX_DETECTION_RANGE_METERS) continue;
    
    // Calculate angle from robot heading to tag
    double angleToTag = Math.atan2(
        tagPos.getY() - robotPos.getY(),
        tagPos.getX() - robotPos.getX()
    );
    double robotHeading = robotPose.getRotation().getRadians();
    
    // TX = angle difference (how far off-center)
    double angleDiff = Math.toDegrees(angleToTag - robotHeading);
    
    // Normalize to -180 to 180
    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    // Check FOV
    if (Math.abs(angleDiff) > HORIZONTAL_FOV_DEG) continue;
    
    // This tag is visible - track if it's the closest
    if (distance < bestDistance) {
        bestDistance = distance;
        bestTagID = tag.ID;
        bestTx = -angleDiff; // Negate for Limelight convention
    }
}
```

The simulation is not pixel-perfect but accurate enough to test alignment logic without a robot.

## Vision-Odometry Fusion

MegaTag2 provides robot pose estimates that can improve odometry accuracy. This is separate from alignment.

### Setup

Wire the consumer in RobotContainer:

```java
limelight.setVisionMeasurementConsumer(
    (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
```

### How It Works

Every loop, if MegaTag2 data is available:

```java
// From LimelightSubsystem.java - updateVisionFusion()
// Set robot orientation for MegaTag2
LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME,
    currentPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

// Get pose estimate
LimelightHelpers.PoseEstimate result =
    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

// Filter bad estimates
if (result == null) return;
if (result.tagCount == 0) return;
if (result.avgTagArea < VisionConstants.MIN_TARGET_AREA) return;

// Reject poses off the field
double x = result.pose.getX();
double y = result.pose.getY();
if (x < 0 || x > 17.0 || y < 0 || y > 9.0) return;

// Feed to drivetrain Kalman filter
visionMeasurementConsumer.accept(result.pose, result.timestampSeconds);
```

The CTRE swerve drivetrain has a built-in Kalman filter that fuses vision measurements with wheel odometry. This improves position accuracy over time.

**Filtering is critical.** Vision measurements with low tag area or impossible positions will corrupt odometry. Always filter before feeding to the drivetrain.

## Tuning Guide

### Initial Setup

1. **Mount the Limelight** - Get it as high as practical, angled up 30-45 degrees
2. **Measure camera height** - Update `CAMERA_HEIGHT_METERS` in VisionConstants
3. **Measure mount angle** - Update `CAMERA_MOUNT_ANGLE_DEGREES`
4. **Set pipeline 0** for AprilTags in Limelight web interface

### Distance Tuning

Test with a real AprilTag at a known distance:

```java
double measuredDistance = limelight.getDistanceMeters();
double actualDistance = // measure with tape measure
double error = measuredDistance - actualDistance;
```

If MegaTag2 distances are consistently off, check:
- Camera height measurement
- Limelight firmware version (update to latest)
- Tag printing quality (must be precise)

### Target Distances

Set the stopping distance for each tag type in VisionConstants:

```java
// Hub shooting distance - start with manufacturer spec
double hubShootDistance = Units.inchesToMeters(97); // 97 inches

// Tower climbing distance - get close to engage hooks
double towerApproachDistance = Units.inchesToMeters(36); // 36 inches
```

Test these in practice and adjust based on your shooter performance and climber reach.

### Control Gains

Start with default values and adjust:

**ROTATION_GAIN (0.03):**
- Too low: Slow, lazy rotation
- Too high: Oscillates back and forth
- Just right: Quick convergence, slight overshoot, settles

**FORWARD_GAIN (0.8):**
- Too low: Takes forever to reach target distance
- Too high: Overshoots and bounces
- Just right: Approaches smoothly, settles within tolerance

**AUTO_STRAFE_GAIN (0.5):**
- Only active in autonomous
- Usually less critical than rotation/forward
- Start low to prevent sideways oscillation

### Speed Limits

Reduce MAX speeds if the robot is too aggressive:

```java
public static final double MAX_ROTATION_SPEED = 0.25;  // 25% of max
public static final double MAX_FORWARD_SPEED = 0.30;   // 30% of max
```

These are multipliers of your drivetrain's maximum speed.

### Alignment Tolerances

Tighten tolerances for precision, loosen for speed:

```java
// Rotation tolerance in degrees
public static final double ALIGNMENT_TOLERANCE_DEGREES = 2.0;

// Distance tolerance in meters (converted from inches)
public static final double DISTANCE_TOLERANCE_METERS = Units.inchesToMeters(3.0);

// Lateral strafe tolerance in meters
public static final double STRAFE_TOLERANCE_METERS = Units.inchesToMeters(3.0);
```

The command finishes when all three are satisfied for ALIGNED_LOOPS_REQUIRED consecutive loops (default 25 loops = 0.5 seconds).

## Common Issues

### "No distance data" on SmartDashboard

**Cause:** MegaTag2 is not providing camera-space coordinates.

**Solutions:**
- Check Limelight is in AprilTag pipeline (pipeline 0)
- Verify firmware is up to date
- Check NetworkTables connection
- Ensure robot orientation is being set (required for MegaTag2)

### Robot doesn't see any tags

**Cause:** Tags are out of FOV or detection range.

**Solutions:**
- Drive closer (max range is 8.0m in sim, varies with lighting in real life)
- Check Limelight LED is on
- Verify pipeline is set to AprilTags, not retroreflective
- Check camera is not obstructed

### Alignment oscillates

**Cause:** Gains too high or tolerances too tight.

**Solutions:**
- Reduce ROTATION_GAIN or FORWARD_GAIN
- Increase ALIGNMENT_TOLERANCE_DEGREES
- Add deadband to controller input if driver interference

### Wrong alliance tags

**Cause:** Alliance not set in DriverStation or hardcoded tag group.

**Solutions:**
- Verify DriverStation shows correct alliance
- Check you're using `VisionConstants::getHubTags` not `BLUE_HUB_TAGS`
- Enable FMS simulation in DriverStation during testing

### Simulation shows no target

**Cause:** Robot pose supplier not set or field layout failed to load.

**Solutions:**
- Verify `limelight.setRobotPoseSupplier()` is called in RobotContainer
- Check console for field layout load errors
- Ensure you're using WPILib 2025 or later (has 2026 field data)

## Integration with Other Systems

### PathPlanner Integration

Register alignment as a named command:

```java
NamedCommands.registerCommand("Align Hub", createAutoHubAlign());
```

Use in paths:
1. Drive near scoring position
2. Run "Align Hub" command
3. Shoot

### Shooter Integration

Auto-aim updates shooter RPM based on distance:

```java
double distance = limelight.getDistanceMeters();
if (distance > 0) {
    shooterSubsystem.setVariableVelocity(
        ShooterConstants.getVelocityForDistance(distance));
}
```

You need to implement `getVelocityForDistance()` based on your shooter characterization data.

## Testing Checklist

**Before Competition:**
- [ ] Test all tag types (hub, tower, outpost)
- [ ] Verify both alliance colors work
- [ ] Check alignment from multiple approach angles
- [ ] Test with driver strafe enabled
- [ ] Verify autonomous alignment completes
- [ ] Confirm vision fusion doesn't corrupt odometry
- [ ] Test timeout behavior
- [ ] Check SmartDashboard values update

**Field Testing:**
- [ ] Calibrate target distances for your shooter
- [ ] Tune gains for actual field lighting
- [ ] Test under both house lights and sunlight
- [ ] Verify performance with bumps/defense
- [ ] Check battery voltage effect on speed limits

## Performance Expectations

With proper tuning:
- **Alignment time:** 1-3 seconds from approach
- **Positional accuracy:** ±3 inches
- **Angular accuracy:** ±2 degrees
- **Vision update rate:** 20-30 Hz (Limelight dependent)
- **Command loop rate:** 50 Hz (20ms period)

## Credits and Resources

**Code Base:** Team 8041 Lake City Ultrabots (2026 season)  
**Limelight Documentation:** https://docs.limelightvision.io/  
**CTRE Phoenix 6:** https://v6.docs.ctr-electronics.com/  
**2026 REBUILT Game Manual:** Official FRC game manual for tag positions  

## Maintenance Notes

**Annual Updates Required:**
- Update field layout for new game (AprilTagFields.kDefaultField)
- Revise tag groups for new field objectives
- Adjust target distances based on new scoring mechanisms
- Re-tune gains if drivetrain changes

**Mid-Season Adjustments:**
- Recalibrate after camera remounting
- Update gains if battery performance degrades
- Adjust tolerances based on competition performance
- Add new tag groups if strategy evolves

## Teaching This System

When teaching other teams or new programmers:

1. **Start with constants** - Show them VisionConstants first so they understand the field layout
2. **Explain camera-space** - Draw diagrams of X/Y/Z axes relative to camera
3. **Walk through one axis** - Rotation is easiest to understand, start there
4. **Add complexity gradually** - Forward, then strafe, then auto-aim
5. **Use simulation** - Let them experiment without risking hardware
6. **Show SmartDashboard** - Live data helps build intuition
7. **Have them tune** - Hands-on tuning teaches more than reading code

The system is complex but each piece is simple. The power comes from combining them correctly.
