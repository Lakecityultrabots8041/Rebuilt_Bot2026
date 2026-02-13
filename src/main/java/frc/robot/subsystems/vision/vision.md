# Vision System — Team 8041

Chris — Lake City Ultrabots, 2026 REBUILT season

This is the full breakdown of how our Limelight vision system works. If you're touching any vision code, read this first.

---

## What the system does

We use a Limelight 4 to see AprilTags on the field. The robot uses those tags to:

- Align itself to scoring positions (hub), climbing positions (tower), and human player stations (outpost)
- Lock heading onto a hub tag while the driver still drives around (auto-aim)
- Feed pose data back into odometry so the drivetrain knows where it is more accurately
- Work in simulation so we can test alignment logic without a real robot

Hardware: Limelight 4, CTRE Phoenix 6 swerve, RoboRIO 2.0.

---

## 2026 REBUILT Field — AprilTag Layout

32 tags total, 36h11 family, IDs 1 through 32. Mirrored across alliances.

### Hub Tags (16 total — 2 per face, 4 faces per hub)

The hub is what we shoot fuel into. It's a 47in x 47in structure centered 158.6in from the alliance wall. The opening is 72in off the carpet.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 2, 3, 4, 5, 8, 9, 10, 11 |
| Red | 18, 19, 20, 21, 24, 25, 26, 27 |

Tag centers sit at **44.25in (1.124m)** off the floor.

### Tower Tags (4 total — 2 per tower)

Towers are for climbing. 49.25in wide, 45in deep, 78.25in tall. Three rungs:
- Low rung: 27.0in (0.686m)
- Mid rung: 45.0in (1.143m)
- High rung: 63.0in (1.600m)

| Alliance | Tag IDs |
|----------|---------|
| Blue | 15, 16 |
| Red | 31, 32 |

Tag centers at **21.75in (0.5525m)** off the floor.

### Outpost Tags (4 total — 2 per outpost)

Outposts are where the human player hands off fuel. The opening is 31.8in wide x 7in tall, bottom at 28.1in off the floor.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 13, 14 |
| Red | 29, 30 |

Tag centers at **21.75in (0.5525m)** off the floor.

### Trench Tags (8 total — 2 per trench)

Each trench has one tag facing the alliance zone and one facing the neutral zone. The trench clearance is 50.34in wide x 22.25in tall — our robot has to fit under it.

| Alliance | Tag IDs |
|----------|---------|
| Blue | 1, 6, 7, 12 |
| Red | 17, 22, 23, 28 |

Tag centers at **35in (0.889m)** off the floor.

### Field Dimensions

- Full field: 317.7in x 651.2in (~8.07m x 16.54m)
- Bump: 6.513in tall, 15-degree ramps
- Depot: 42in wide x 27in deep, walls ~1.125in tall

---

## File Breakdown

**LimelightSubsystem.java** — talks to the Limelight hardware (or fakes it in sim). Reads NetworkTables, caches MegaTag2 distance/lateral data, feeds vision poses into drivetrain odometry.

**VisionConstants.java** — all the numbers. Tag groups, target distances, PID gains, speed limits, tolerances. Alliance-aware getter methods live here too.

**Limelight_Move.java** — the alignment command. Controls rotation, forward/back, and strafe simultaneously. Works with tag groups (not single IDs). Has two strafe modes: driver input for teleop, auto-correction for autonomous.

**RobotContainer.java** — wires everything together. Creates command instances, sets up button bindings, configures auto-aim in the default drive command.

---

## How Distance Works

We don't use the old trig method (measuring pixel height and doing math with camera angle). We use MegaTag2's camera-space coordinates, which give us the tag's 3D position relative to the camera directly in meters.

The camera-space array from Limelight has three axes:
- **Index 0 (X):** lateral offset — positive means the tag is to the right of center
- **Index 1 (Y):** vertical offset — positive means above center
- **Index 2 (Z):** forward distance — how far away the tag is

In `LimelightSubsystem.updateMegaTag2Cache()`:

```java
double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(LIMELIGHT_NAME);
if (targetPose != null && targetPose.length >= 3 && tv.getDouble(0) == 1) {
    cachedLateralOffsetMeters = targetPose[0]; // X = lateral
    cachedDistanceMeters = targetPose[2];      // Z = forward
    megatag2Available = true;
}
```

This is better than trig because Limelight's 3D pose solver already accounts for camera angle, tag size, and perspective. Less tuning, more accurate.

All internal math is in **meters**. We only convert to inches for SmartDashboard display. Mixing units is a classic bug source — don't do it.

---

## Alliance-Aware Tag Selection

The field is mirrored, so red and blue have their own set of tags for each structure. We need to align to OUR tags, not theirs.

`VisionConstants` has getter methods that check DriverStation for the current alliance:

```java
public static Set<Integer> getHubTags() {
    return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
}
```

The key thing: these get called when the command **starts**, not when the command object is created. This matters because in autonomous, you might not know your alliance color until the match begins.

In RobotContainer, the command factories use method references:

```java
private Limelight_Move createHubAlign() {
    return new Limelight_Move(drivetrain, limelight,
        VisionConstants::getHubTags, () -> -controller.getLeftX());
}
```

`VisionConstants::getHubTags` passes the method itself as a Supplier. Every time `Limelight_Move.initialize()` runs, it calls `.get()` on that supplier and gets fresh alliance data.

**Don't do this:**
```java
// WRONG — evaluates once at robot boot, locks to whatever alliance was set at that moment
new Limelight_Move(drivetrain, limelight, () -> VisionConstants.BLUE_HUB_TAGS, strafeSupplier);
```

---

## Three-Axis Alignment (Limelight_Move)

The command controls rotation, forward/back, and strafe all at once. Each axis uses proportional control with clamped output.

### Rotation

Uses TX (horizontal offset in degrees from Limelight). Positive TX means the tag is to the right, so we apply negative rotation to turn toward it.

```java
double rotationOutput = -horizontalErrorDeg * VisionConstants.ROTATION_GAIN;
rotationOutput = MathUtil.clamp(rotationOutput, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
double rotationVelocityRps = rotationOutput * maxAngularRateRps;
```

ROTATION_GAIN is 0.03. If rotation feels sluggish, bump it up. If it oscillates (wobbles back and forth), bring it down.

### Forward/Back

Uses the camera-space Z distance. Error = current distance minus target distance. Positive error means we're too far, so we drive forward.

```java
double distanceErrorMeters = currentDistanceMeters - targetDistanceMeters;
double forwardOutput = distanceErrorMeters * VisionConstants.FORWARD_GAIN;
forwardOutput = MathUtil.clamp(forwardOutput, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
```

FORWARD_GAIN is 0.8 — higher than rotation because distance errors tend to be bigger in magnitude.

### Strafe

This one has two modes:

**Teleop** — if the driver is pushing the stick left/right, that input takes priority. This lets the driver arc around while the system handles rotation and distance. Useful for dodging defense.

**Autonomous** — if there's no driver input, the system auto-corrects lateral offset using camera-space X data. This centers the robot on the tag.

```java
if (Math.abs(driverStrafe) > 0.0) {
    strafeVelocityMps = driverStrafe * maxSpeedMps * MAX_DRIVER_STRAFE_SCALE;
} else {
    double strafeOutput = lateralOffsetMeters * AUTO_STRAFE_GAIN;
    strafeOutput = MathUtil.clamp(strafeOutput, -MAX_AUTO_STRAFE_SPEED, MAX_AUTO_STRAFE_SPEED);
    strafeVelocityMps = strafeOutput * maxSpeedMps;
}
```

### When is it "aligned"?

The command checks three conditions every loop:
- Rotation error < ALIGNMENT_TOLERANCE_DEGREES (2.0 deg)
- Distance error < DISTANCE_TOLERANCE_METERS (3 inches)
- Lateral error < STRAFE_TOLERANCE_METERS (3 inches)

All three must be satisfied for ALIGNED_LOOPS_REQUIRED consecutive loops (25 loops = ~0.5 seconds at 50Hz). This prevents the command from ending on a single good reading — it has to hold steady.

If it can't align within ALIGNMENT_TIMEOUT_SECONDS (8s), it gives up.

---

## Auto-Aim

Auto-aim is different from Limelight_Move. Instead of a separate command that takes over the drivetrain, auto-aim lives inside the drivetrain's default command. The driver keeps full translational control (left stick), but rotation gets overridden by a PID that locks heading onto the hub tag.

When auto-aim is active and a hub tag is visible:

1. Get the current robot heading and TX offset
2. Compute the target heading (current heading minus TX in radians)
3. PID calculates rotation output
4. Clamp it to AUTO_AIM_MAX_ROTATION_RATE (1.5 rad/s)
5. Run it through a slew rate limiter (3.0 rad/s²) to smooth acceleration
6. Update shooter RPM based on distance

```java
double currentHeading = drivetrain.getState().Pose.getRotation().getRadians();
double targetHeading = currentHeading - Math.toRadians(limelight.getHorizontalOffset());
double rawOutput = autoAimPID.calculate(currentHeading, targetHeading);
double clampedOutput = MathUtil.clamp(rawOutput, -AUTO_AIM_MAX_ROTATION_RATE, AUTO_AIM_MAX_ROTATION_RATE);
double smoothOutput = autoAimSlew.calculate(clampedOutput);
```

The PID uses continuous input (`enableContinuousInput(-PI, PI)`) because heading wraps around. Without this, the PID would try to rotate 359 degrees instead of 1 degree the other way.

When auto-aim deactivates (no hub tag visible, or toggled off), we reset the slew limiter and PID so they don't carry stale state into the next activation. If the shooter was in VARIABLE mode, we zero it out.

Auto-aim only fires when:
- It's toggled on (left bumper)
- A hub tag is visible
- Distance is positive and under 5.0m

---

## Vision-Odometry Fusion

Separate from alignment — this is about making odometry more accurate over time.

Every loop, if MegaTag2 data is good, we feed the pose estimate into the CTRE swerve drivetrain's Kalman filter. The filter blends vision with wheel odometry.

Setup in RobotContainer:
```java
limelight.setVisionMeasurementConsumer(
    (pose, timestamp) -> drivetrain.addVisionMeasurement(pose, timestamp));
```

Before feeding a pose, we filter it hard:
- Must have at least 1 tag visible
- Tag area must be above MIN_TARGET_AREA (0.1) — rejects tiny far-away detections
- Pose X/Y must be on the actual field (0–17m x 0–9m) — rejects garbage estimates

We also set the robot's current heading via `SetRobotOrientation()` before pulling MegaTag2 data, because MegaTag2 needs to know the robot's orientation to give a good blue-origin pose.

Bad vision data will corrupt your odometry. Always filter.

---

## Simulation

When `RobotBase.isSimulation()` is true, LimelightSubsystem skips NetworkTables entirely and fakes the data using the robot's simulated pose and the official 2026 field layout.

It loops through every tag on the field and checks:
1. Is the tag within 8.0m?
2. Is the tag within the camera's horizontal FOV (29.8 degrees half-width)?
3. If multiple tags pass, pick the closest one

Then it generates fake TX, distance, area, and lateral offset values. The area is approximated as `10 / distance²` — not precise but good enough. TY is left at 0.

You need to call `limelight.setRobotPoseSupplier(() -> drivetrain.getState().Pose)` in RobotContainer for sim to work. Without a pose supplier, sim mode has nothing to calculate from.

The sim isn't pixel-perfect but it lets you test alignment logic, alliance selection, and command flow without a robot.

---

## Target Distances

Each tag type has a target stop distance in VisionConstants (the `APRILTAG_DISTANCES` array, indexed by tag ID):

| Structure | Distance | Why |
|-----------|----------|-----|
| Hub | 97in (~2.46m) | Shooting range — tune for your shooter |
| Tower | 36in (~0.91m) | Close enough to engage climbing rungs |
| Outpost | 30in (~0.76m) | Close for human player handoff |
| Trench | 48in (~1.22m) | Center under the trench arm |

These are starting points. Tune them on the real field based on shooter performance and mechanism reach.

---

## Tuning

### Gains

| Constant | Default | Too low | Too high |
|----------|---------|---------|----------|
| ROTATION_GAIN | 0.03 | Slow, lazy turning | Oscillates side to side |
| FORWARD_GAIN | 0.8 | Takes forever to reach distance | Overshoots and bounces |
| AUTO_STRAFE_GAIN | 0.5 | Doesn't correct lateral drift | Sideways oscillation |

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
- Slew rate: 3.0 rad/s²

The D term adds damping so it doesn't overshoot the target heading. The slew limiter prevents sudden jerks.

### Camera Setup

Measure these on the actual robot and update VisionConstants:
- CAMERA_HEIGHT_METERS — lens height from floor
- CAMERA_MOUNT_ANGLE_DEGREES — 0 = horizontal, positive = tilted up

Mount the Limelight as high as you can, angled up 30–45 degrees. Set pipeline 0 for AprilTags.

---

## Controller Bindings

| Button | Action |
|--------|--------|
| Left stick | Drive (X/Y translation) |
| Right stick X | Rotate (when auto-aim is off) |
| Right bumper | Reset field-centric heading |
| Left bumper | Toggle auto-aim |
| START | Align to hub |
| Y | Align to tower |
| X | Align to outpost |
| Right trigger | Shoot |
| Left trigger | Eject |

---

## PathPlanner Integration

Named commands registered for autonomous:
- `"Align Hub"` — runs `createAutoHubAlign()` (no driver strafe)
- `"Rev Shooter"`, `"Shoot"`, `"Idle Shooter"` — shooter control
- `"AlignAndShoot"` — sequences hub align then shoot

Use these in PathPlanner paths: drive near the target, run the named command, continue.

---

## Common Problems

**No distance data on SmartDashboard** — MegaTag2 isn't returning camera-space data. Check that pipeline 0 is set, firmware is current, NetworkTables is connected, and `SetRobotOrientation` is being called.

**Robot doesn't see tags** — too far away, camera obstructed, wrong pipeline, or LEDs off. Sim max range is 8.0m, real range depends on lighting.

**Alignment oscillates** — gains too high or tolerances too tight. Lower ROTATION_GAIN / FORWARD_GAIN first. If the driver is accidentally providing input during alignment, check your deadband.

**Wrong alliance tags** — DriverStation doesn't have alliance set, or you hardcoded a tag set instead of using the method reference getters. Always use `VisionConstants::getHubTags`, never `BLUE_HUB_TAGS` directly.

**Sim shows nothing** — `setRobotPoseSupplier()` not called, or field layout failed to load. Check console for errors.

---

## Testing Checklist

Before competition:
- [ ] All tag types work (hub, tower, outpost)
- [ ] Both alliance colors work
- [ ] Alignment from multiple approach angles
- [ ] Driver strafe during alignment
- [ ] Autonomous alignment completes
- [ ] Vision fusion doesn't corrupt odometry
- [ ] Timeout behavior
- [ ] SmartDashboard values updating

On the real field:
- [ ] Calibrate target distances
- [ ] Tune gains for field lighting
- [ ] Test in different lighting (gym lights, sunlight)
- [ ] Performance with bumps and defense
- [ ] Battery voltage effects

---

Team 8041 — Lake City Ultrabots, 2026
