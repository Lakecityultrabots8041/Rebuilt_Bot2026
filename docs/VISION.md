# Vision System - Team 8041

Chris Shaw - Lake City Ultrabots Programming Mentor
2026 REBUILT season

This is the full breakdown of how our Limelight vision system works. If you are touching any vision code, read this first.

---

## What the system does

We use two Limelight 4 cameras to see AprilTags on the field. The robot uses those tags to:

- Align itself to scoring positions (hub), climbing positions (tower), and human player stations (outpost)
- Lock heading onto a hub tag while the driver still drives around (auto-aim)
- Feed pose data from both cameras into odometry so the drivetrain knows where it is more accurately
- Work in simulation so we can test alignment logic without a real robot

Hardware: two Limelight 4 cameras, CTRE Phoenix 6 swerve, RoboRIO 2.0.

### Two Camera Setup

| Camera | NetworkTables Name | Location | Faces | Used For |
|--------|-------------------|----------|-------|----------|
| Shooter camera | `limelight-april` | Near the shooter | Shooter side | Hub/outpost/trench alignment, auto-aim |
| Intake camera | `limelight-intake` | Near the intake | Intake side | Pose fusion, future intake-side alignment |

Both cameras run independently. Both fuse their pose estimates into the drivetrain Kalman filter every loop, which means better localization from two different vantage points. The intake camera does not currently run any alignment commands, but the code supports it if needed later.

**Note about Pigeon orientation:** The Pigeon IMU's "forward" direction is the climber/intake side of the robot, not the shooter side. This is a quirk of how it was installed. The driver uses the right bumper (reset field-centric heading) to fix orientation during a match. The code does not try to compensate for this. It just works with whatever the driver sets as forward.

---

## 2026 REBUILT Field - AprilTag Layout

32 tags total, 36h11 family, IDs 1 through 32. Mirrored across alliances.

### Hub Tags (16 total, 2 per face, 4 faces per hub)

The hub is what we shoot fuel into. It is a 47in x 47in structure centered 158.6in from the alliance wall. The opening is 72in off the carpet.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 2, 3, 4, 5, 8, 9, 10, 11 |
| Red | 18, 19, 20, 21, 24, 25, 26, 27 |

Tag centers sit at 44.25in (1.124m) off the floor.

### Tower Tags (4 total, 2 per tower)

Towers are for climbing. 49.25in wide, 45in deep, 78.25in tall. Three rungs:
- Low rung: 27.0in (0.686m)
- Mid rung: 45.0in (1.143m)
- High rung: 63.0in (1.600m)

| Alliance | Tag IDs |
|----------|---------|
| Blue | 15, 16 |
| Red | 31, 32 |

Tag centers at 21.75in (0.5525m) off the floor.

### Outpost Tags (4 total, 2 per outpost)

Outposts are where the human player hands off fuel. The opening is 31.8in wide x 7in tall, bottom at 28.1in off the floor.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 13, 14 |
| Red | 29, 30 |

Tag centers at 21.75in (0.5525m) off the floor.

### Trench Tags (8 total, 2 per trench)

Each trench has one tag facing the alliance zone and one facing the neutral zone. The trench clearance is 50.34in wide x 22.25in tall.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 1, 6, 7, 12 |
| Red | 17, 22, 23, 28 |

Tag centers at 35in (0.889m) off the floor.

### Field Dimensions

- Full field: 317.7in x 651.2in (about 8.07m x 16.54m)
- Bump: 6.513in tall, 15-degree ramps
- Depot: 42in wide x 27in deep, walls about 1.125in tall

---

## File Breakdown

**LimelightSubsystem.java**: Talks to a single Limelight camera via NetworkTables (or fakes it in sim). The constructor takes two arguments: the camera name (e.g. `"limelight-april"`) and whether it faces the rear of the robot. Each instance caches its own MegaTag2 distance/lateral data and feeds vision poses into drivetrain odometry independently.

**VisionConstants.java**: All the numbers. Tag groups, target distances, PID gains, speed limits, tolerances, and camera mounting values for both cameras. Alliance-aware getter methods live here too.

**Limelight_Move.java**: The alignment command. Controls rotation, forward/back, and strafe simultaneously. Works with tag groups (not single IDs). Has a direction multiplier that flips all outputs when using a rear-facing camera, so the robot drives toward what that camera sees. Two strafe modes: driver input for teleop, auto-correction for autonomous.

**RobotContainer.java**: Wires everything together. Creates both camera instances, sets up pose fusion for both, builds command factories, configures button bindings, and runs auto-aim in the default drive command.

---

## How the Subsystem Supports Two Cameras

`LimelightSubsystem` is parameterized, not hardcoded to one camera. The constructor:

```java
public LimelightSubsystem(String cameraName, boolean rearFacing)
```

- `cameraName` sets the NetworkTables table name and the SmartDashboard prefix
- `rearFacing` marks whether this camera faces the opposite direction from the robot's drive-forward

In RobotContainer, two instances are created:

```java
// Shooter camera, faces the shooter side
private final LimelightSubsystem limelightShooter =
    new LimelightSubsystem("limelight-april", false);

// Intake camera, faces the intake side
private final LimelightSubsystem limelightIntake =
    new LimelightSubsystem("limelight-intake", true);
```

Both cameras get their own SmartDashboard entries. The shooter camera publishes under `limelight-april/Has Target`, `limelight-april/Distance (m)`, etc. The intake camera publishes under `limelight-intake/Has Target`, `limelight-intake/Distance (m)`, etc. No key collisions.

Both cameras fuse poses independently:

```java
limelightShooter.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
limelightShooter.setVisionMeasurementConsumer(
    (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));

limelightIntake.setRobotPoseSupplier(() -> drivetrain.getState().Pose);
limelightIntake.setVisionMeasurementConsumer(
    (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
```

The CTRE swerve Kalman filter accepts multiple measurement sources naturally. More data from different angles means better odometry.

---

## How Distance Works

We do not use the old trig method (measuring pixel height and doing math with camera angle). We use MegaTag2's camera-space coordinates, which give us the tag's 3D position relative to the camera directly in meters.

The camera-space array from Limelight has three axes:
- **Index 0 (X):** lateral offset, positive means the tag is to the right of center
- **Index 1 (Y):** vertical offset, positive means above center
- **Index 2 (Z):** forward distance, how far away the tag is

In `LimelightSubsystem.updateMegaTag2Cache()`:

```java
double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
if (targetPose != null && targetPose.length >= 3 && cachedHasTarget) {
    cachedLateralOffsetMeters = targetPose[0]; // X = lateral
    cachedDistanceMeters = targetPose[2];      // Z = forward
    megatag2Available = true;
}
```

This is better than trig because Limelight's 3D pose solver already accounts for camera angle, tag size, and perspective. Less tuning, more accurate.

All internal math is in meters. We only convert to inches for SmartDashboard display. Mixing units is a classic bug source, do not do it.

---

## Alliance-Aware Tag Selection

The field is mirrored, so red and blue have their own set of tags for each structure. We need to align to OUR tags, not theirs.

`VisionConstants` has getter methods that check DriverStation for the current alliance:

```java
public static Set<Integer> getHubTags() {
    return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
}
```

These get called when the command **starts**, not when the command object is created. This matters because in autonomous, you might not know your alliance color until the match begins.

In RobotContainer, the command factories use method references:

```java
private Limelight_Move createHubAlign() {
    return new Limelight_Move(drivetrain, limelightShooter,
        VisionConstants::getHubTags, () -> -controller.getLeftX());
}
```

`VisionConstants::getHubTags` passes the method itself as a Supplier. Every time `Limelight_Move.initialize()` runs, it calls `.get()` on that supplier and gets fresh alliance data.

**Do not do this:**
```java
// WRONG - evaluates once at robot boot, locks to whatever alliance was set at that moment
new Limelight_Move(drivetrain, limelightShooter, () -> VisionConstants.BLUE_HUB_TAGS, strafeSupplier);
```

---

## Three-Axis Alignment (Limelight_Move)

The command controls rotation, forward/back, and strafe all at once. Each axis uses proportional control with clamped output.

### Direction Multiplier (Front vs Rear Camera)

`Limelight_Move` has a `directionMultiplier` field that is `+1.0` for a front-facing camera and `-1.0` for a rear-facing camera. This flips all three drive outputs so the robot drives toward what the rear camera sees (effectively driving backward).

```java
this.directionMultiplier = limelight.isRearFacing() ? -1.0 : 1.0;
```

The multiplier is applied to the final velocity for rotation, forward, and strafe. The alignment tolerance checks stay the same regardless of camera direction.

Currently all alignment commands use `limelightShooter` (front-facing, multiplier = +1.0). If the intake camera is ever used for alignment, the multiplier handles it automatically.

### Rotation

Uses TX (horizontal offset in degrees from Limelight). Positive TX means the tag is to the right, so we apply negative rotation to turn toward it.

```java
double rotationOutput = -horizontalErrorDeg * VisionConstants.ROTATION_GAIN;
rotationOutput = MathUtil.clamp(rotationOutput, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
double rotationVelocityRps = rotationOutput * maxAngularRateRps * directionMultiplier;
```

ROTATION_GAIN is 0.03. If rotation feels sluggish, bump it up. If it oscillates (wobbles back and forth), bring it down.

### Forward/Back

Uses the camera-space Z distance. Error = current distance minus target distance. Positive error means we are too far, so we drive forward.

```java
double distanceErrorMeters = currentDistanceMeters - targetDistanceMeters;
double forwardOutput = distanceErrorMeters * VisionConstants.FORWARD_GAIN;
forwardOutput = MathUtil.clamp(forwardOutput, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
double forwardVelocityMps = forwardOutput * maxSpeedMps * directionMultiplier;
```

FORWARD_GAIN is 0.8, higher than rotation because distance errors tend to be bigger in magnitude.

### Strafe

This one has two modes:

**Teleop** - if the driver is pushing the stick left/right, that input takes priority. This lets the driver arc around while the system handles rotation and distance. Useful for dodging defense.

**Autonomous** - if there is no driver input, the system auto-corrects lateral offset using camera-space X data. This centers the robot on the tag.

```java
if (Math.abs(driverStrafe) > 0.0) {
    strafeVelocityMps = driverStrafe * maxSpeedMps * MAX_DRIVER_STRAFE_SCALE
        * directionMultiplier;
} else {
    double strafeOutput = lateralOffsetMeters * AUTO_STRAFE_GAIN;
    strafeOutput = MathUtil.clamp(strafeOutput, -MAX_AUTO_STRAFE_SPEED, MAX_AUTO_STRAFE_SPEED);
    strafeVelocityMps = strafeOutput * maxSpeedMps * directionMultiplier;
}
```

### When is it "aligned"?

The command checks three conditions every loop:
- Rotation error < ALIGNMENT_TOLERANCE_DEGREES (2.0 deg)
- Distance error < DISTANCE_TOLERANCE_METERS (3 inches)
- Lateral error < STRAFE_TOLERANCE_METERS (3 inches)

All three must be satisfied for ALIGNED_LOOPS_REQUIRED consecutive loops (25 loops = about 0.5 seconds at 50Hz). This prevents the command from ending on a single good reading, it has to hold steady.

If it cannot align within ALIGNMENT_TIMEOUT_SECONDS (8s), it gives up.

---

## Auto-Aim

Auto-aim is different from Limelight_Move. Instead of a separate command that takes over the drivetrain, auto-aim lives inside the drivetrain's default command. The driver keeps full translational control (left stick), but rotation gets overridden by a PID that locks heading onto the hub tag.

Auto-aim only uses the **shooter camera** (`limelightShooter`). The intake camera is not involved.

When auto-aim is active and a hub tag is visible:

1. Get the current robot heading and TX offset
2. Compute the target heading (current heading minus TX in radians)
3. PID calculates rotation output
4. Clamp it to AUTO_AIM_MAX_ROTATION_RATE (1.5 rad/s)
5. Run it through a slew rate limiter (3.0 rad/s^2) to smooth acceleration
6. Update shooter RPM based on distance

```java
double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
double targetHeading = currentHeading - Math.toRadians(limelightShooter.getHorizontalOffset());
double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
double clampedOutput = MathUtil.clamp(rawOutput, -AUTO_AIM_MAX_ROTATION_RATE, AUTO_AIM_MAX_ROTATION_RATE);
double smoothOutput = autoAimSlew.calculate(clampedOutput);
```

The PID uses continuous input (`enableContinuousInput(-PI, PI)`) because heading wraps around. Without this, the PID would try to rotate 359 degrees instead of 1 degree the other way.

When auto-aim deactivates (no hub tag visible, or toggled off), we reset the slew limiter and PID so they do not carry stale state into the next activation.

Auto-aim only fires when:
- It is toggled on (left bumper)
- A hub tag is visible
- Distance is positive and under 5.0m

---

## Vision-Odometry Fusion

This is separate from alignment. It is about making odometry more accurate over time.

Every loop, if MegaTag2 data is good, we feed the pose estimate into the CTRE swerve drivetrain's Kalman filter. The filter blends vision with wheel odometry. **Both cameras do this independently**, so the robot gets pose corrections from both the shooter side and the intake side.

Before feeding a pose, we filter it:
- Must have at least 1 tag visible
- Tag area must be above MIN_TARGET_AREA (0.1), rejects tiny far-away detections
- Pose X/Y must be on the actual field (0-17m x 0-9m), rejects garbage estimates

We also set the robot's current heading via `SetRobotOrientation()` before pulling MegaTag2 data, because MegaTag2 needs to know the robot's orientation to give a good blue-origin pose.

Bad vision data will corrupt your odometry. Always filter.

---

## Simulation

When `RobotBase.isSimulation()` is true, LimelightSubsystem skips NetworkTables entirely and fakes the data using the robot's simulated pose and the official 2026 field layout.

It loops through every tag on the field and checks:
1. Is the tag within 8.0m?
2. Is the tag within the camera's horizontal FOV (29.8 degrees half-width)?
3. If multiple tags pass, pick the closest one

For rear-facing cameras (`isRearFacing = true`), the simulation offsets the robot heading by 180 degrees before checking FOV. This means the intake camera sees tags behind the robot, and the shooter camera sees tags in front.

Then it generates fake TX, distance, area, and lateral offset values. The area is approximated as `10 / distance^2`, not precise but good enough. TY is left at 0.

You need to call `setRobotPoseSupplier()` on **both** camera instances in RobotContainer for sim to work. Without a pose supplier, sim mode has nothing to calculate from.

The sim is not pixel-perfect but it lets you test alignment logic, alliance selection, and command flow without a robot.

---

## Target Distances

Each tag type has a target stop distance in VisionConstants (the `APRILTAG_DISTANCES` array, indexed by tag ID):

| Structure | Distance | Why |
|-----------|----------|-----|
| Hub | 97in (~2.46m) | Shooting range, tune for your shooter |
| Tower | 18in (~0.46m) | Close approach for climbing, near bumper contact |
| Outpost | 14in (~0.36m) | Very close for human player handoff |
| Trench | 48in (~1.22m) | Center under the trench arm |

These are starting points. Tune them on the real field based on shooter performance and mechanism reach. Tower and outpost distances are intentionally close because the robot needs to be right up against those structures.

---

## Tuning

### Gains

| Constant | Default | Too low | Too high |
|----------|---------|---------|----------|
| ROTATION_GAIN | 0.03 | Slow lazy turning | Oscillates side to side |
| FORWARD_GAIN | 0.8 | Takes forever to reach distance | Overshoots and bounces |
| AUTO_STRAFE_GAIN | 0.5 | Does not correct lateral drift | Sideways oscillation |

### Speed Limits

All values are fractions of max drivetrain speed (0.0 to 1.0):
- MAX_ROTATION_SPEED: 0.25
- MAX_FORWARD_SPEED: 0.30
- MAX_DRIVER_STRAFE_SCALE: 0.5
- MAX_AUTO_STRAFE_SPEED: 0.20

If the robot is too aggressive during alignment, drop these.

### Auto-Aim PID

- KP = 4.0, KI = 0.0, KD = 0.1
- Max rotation rate: 1.5 rad/s
- Slew rate: 3.0 rad/s^2

The D term adds damping so it does not overshoot the target heading. The slew limiter prevents sudden jerks.

### Camera Setup

**Both cameras have placeholder mounting values that must be measured on the actual robot:**

| Camera | Height | Angle | Constant Prefix |
|--------|--------|-------|-----------------|
| Shooter ("limelight-april") | 25.125in (placeholder) | 0 deg (placeholder) | `FRONT_CAMERA_` |
| Intake ("limelight-intake") | 25.125in (placeholder) | 0 deg (placeholder) | `REAR_CAMERA_` |

Measure from the floor to the center of each Limelight lens. The angle is 0 for horizontal, positive for tilted up. Mount cameras as high as you can, angled up 30-45 degrees. Set pipeline 0 for AprilTags.

These values are used by the trig-based backup calculation and by simulation, not by MegaTag2 (which handles camera pose internally).

---

## Controller Bindings (Vision Related)

| Button | Action |
|--------|--------|
| Left Bumper | Toggle auto-aim |
| Right Bumper | Reset field-centric heading |
| START | Align to hub (hold) |
| BACK | Align to tower (hold) |
| Y | Align to outpost (hold) |
| D-Left | Follow any tag demo (hold) |

See `docs/CONTROLLER_MAP.md` for the full binding table.

---

## PathPlanner Named Commands

### Standalone Alignment (robot drives to target, finishes when aligned)

| Name | Camera | Tag Group |
|------|--------|-----------|
| `Align Hub` | Shooter | Alliance hub tags |
| `Align Outpost` | Shooter | Alliance outpost tags |
| `Align Tower` | Shooter | Alliance tower tags |
| `Align Trench` | Shooter | Alliance trench tags |

### Align-Then-Shoot Combos (align, then full shoot sequence)

| Name | What it does |
|------|-------------|
| `AlignAndShoot` | Hub align, then shoot sequence |
| `AlignOutpostAndShoot` | Outpost align, then shoot sequence |
| `AlignTowerAndShoot` | Tower align, then shoot sequence |

### Shooter and Intake Commands

| Name | What it does |
|------|-------------|
| `Rev Shooter` | Pre-spin flywheel |
| `Shoot` | Flywheel + feed on |
| `Idle Shooter` | Everything off |
| `Pass` | Pass speed sequence |
| `Intake` | Intake roller on |
| `Eject` | Intake eject |
| `Idle Intake` | Intake roller off |
| `Pivot To Stow` | Arm to stow position |
| `Pivot To Intake` | Arm to intake position |
| `Pivot To Travel` | Arm to travel position |
| `Start Intake` | Deploy arm, then start roller |
| `End Intake` | Stop roller, travel position |

Use these in PathPlanner autos. Typical pattern: drive near a target with a path, run an alignment or intake named command, continue with the next path.

**Pro tip:** Use an event marker to start `Rev Shooter` at 60% through the path before an `AlignAndShoot`. This pre-spins the flywheel while still driving, saving time.

---

## Common Problems

**No distance data on SmartDashboard**: MegaTag2 is not returning camera-space data. Check that pipeline 0 is set, firmware is current, NetworkTables is connected, and `SetRobotOrientation` is being called. Make sure you are looking at the correct camera prefix (`limelight-april/` or `limelight-intake/`).

**Robot does not see tags**: Too far away, camera obstructed, wrong pipeline, or LEDs off. Sim max range is 8.0m, real range depends on lighting.

**Alignment oscillates**: Gains too high or tolerances too tight. Lower ROTATION_GAIN / FORWARD_GAIN first. If the driver is accidentally providing input during alignment, check your deadband.

**Wrong alliance tags**: DriverStation does not have alliance set, or you hardcoded a tag set instead of using the method reference getters. Always use `VisionConstants::getHubTags`, never `BLUE_HUB_TAGS` directly.

**Sim shows nothing**: `setRobotPoseSupplier()` not called on the camera instance you are testing, or field layout failed to load. Check console for errors.

**Only one camera shows data**: Each camera is a separate subsystem instance. Make sure both are constructed in RobotContainer and both have their pose supplier and measurement consumer set.

**Dashboard keys changed**: With two cameras, the SmartDashboard prefix is now the camera name (e.g. `limelight-april/Has Target`) instead of the old `Limelight/Has Target`. Update your Elastic/Shuffleboard layouts.

---

## Testing Checklist

Before competition:
- [ ] Both cameras show data on SmartDashboard under their own prefix
- [ ] All tag types work (hub, tower, outpost, trench)
- [ ] Both alliance colors work
- [ ] Alignment from multiple approach angles
- [ ] Driver strafe during alignment
- [ ] Autonomous alignment completes via named commands
- [ ] Vision fusion from both cameras does not corrupt odometry
- [ ] Timeout behavior (8 second limit)
- [ ] SmartDashboard values updating for both cameras

On the real field:
- [ ] Measure and update camera mounting values for both cameras
- [ ] Calibrate target distances (especially tower 18in and outpost 14in)
- [ ] Tune gains for field lighting
- [ ] Test in different lighting (gym lights, sunlight)
- [ ] Performance with bumps and defense
- [ ] Battery voltage effects

---

Team 8041 - Lake City Ultrabots, 2026
