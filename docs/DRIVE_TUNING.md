# Drive Tuning Guide

Teleop tuning values: `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`
Swerve module PIDs: `src/main/java/frc/robot/generated/TunerConstants.java`
Deadband config: `RobotContainer.java` (drive request setup)

---

## Asymmetric Slew Rate Limiting

The robot uses asymmetric slew rate limiting on the two translation axes (forward/back and strafe). Acceleration and deceleration are limited independently, so you can have a fast snappy feel without violent stops that tip the robot.

- `MAX_TELEOP_ACCEL` - how fast the commanded speed ramps up
- `MAX_TELEOP_DECEL` - how fast the commanded speed ramps down

Tune acceleration freely. Tune deceleration carefully. Too high of a decel value is what causes tipping on a top-heavy robot.

### Timing Reference at Full Speed (3.79 m/s)

| Value | 0 to full speed | Full speed to stop |
|---|---|---|
| 3.0 | 1.53 s | 1.53 s |
| 4.0 | 1.15 s | 1.15 s |
| 5.0 | 0.92 s | 0.92 s |
| 6.0 | 0.76 s | 0.76 s |
| 8.0 | 0.57 s | 0.57 s |
| 10.0 | 0.46 s | 0.46 s |
| 12.0 | 0.38 s | 0.38 s |

### Recommended Tuning Process

1. Set `MAX_TELEOP_ACCEL = 8.0` and leave it there unless the robot feels loose.
2. Start `MAX_TELEOP_DECEL = 4.0`.
3. Drive at full speed and release the stick abruptly. If the robot tips, lower the decel value.
4. If the robot stays planted, increase decel by 0.5 and repeat.
5. Stop when you find the highest decel value that does not tip the robot on carpet.

---

## How applyDriveSlew Works (Code Explanation)

This section explains the `applyDriveSlew` method in `RobotContainer.java` for students who haven't seen this style of Java before.

```java
private double applyDriveSlew(double current, double requested) {
    double rateLimit = (Math.abs(requested) > Math.abs(current))
        ? DriveConstants.MAX_TELEOP_ACCEL
        : DriveConstants.MAX_TELEOP_DECEL;
    double maxDelta = rateLimit * 0.02;
    return current + MathUtil.clamp(requested - current, -maxDelta, maxDelta);
}
```

### The ? and : (Ternary Operator)

The `?` and `:` together are called the ternary operator. It is a shorthand way of writing an if/else that produces a value.

The pattern is:

```
condition ? valueIfTrue : valueIfFalse
```

So this line:

```java
double rateLimit = (Math.abs(requested) > Math.abs(current))
    ? DriveConstants.MAX_TELEOP_ACCEL
    : DriveConstants.MAX_TELEOP_DECEL;
```

Is exactly the same as writing:

```java
double rateLimit;
if (Math.abs(requested) > Math.abs(current)) {
    rateLimit = DriveConstants.MAX_TELEOP_ACCEL;
} else {
    rateLimit = DriveConstants.MAX_TELEOP_DECEL;
}
```

If the speed the driver is asking for (in any direction) is larger than what the robot is currently doing, we are speeding up and use the acceleration limit. Otherwise we are slowing down or reversing and use the deceleration limit.

`Math.abs()` strips the sign so we are comparing magnitudes, not direction. It works the same whether the robot is going forward, backward, or strafing.

### The Rest of the Method

```java
double maxDelta = rateLimit * 0.02;
```

The robot loop runs every 20 milliseconds (0.02 seconds). Multiplying the rate by 0.02 gives the maximum speed change allowed in a single loop cycle.

```java
return current + MathUtil.clamp(requested - current, -maxDelta, maxDelta);
```

`requested - current` is how much the speed needs to change this cycle. `MathUtil.clamp` caps that change to at most `maxDelta` in either direction. The result is added to the current speed, so the speed steps toward the target without jumping there instantly.

---

## Drive Motor Current Limits

Drive motors have two independent current limits. Both survive a Tuner X regeneration because they are applied programmatically in `CommandSwerveDrivetrain.configureDriveMotors()`, not in `TunerConstants.java`.

### Stator Current Limit (`DRIVE_STATOR_CURRENT_LIMIT_AMPS`)

Limits the torque each drive motor can produce. Lower values reduce wheel slam on hard stops and reduce battery sag under acceleration.

Typical starting range for Kraken X60: 60-120 A

- Too low: robot feels sluggish accelerating, wheels may slip before reaching the limit
- Too high: violent wheel slam, more battery sag, higher tipping risk on hard braking

### Supply Current Limit (`DRIVE_SUPPLY_CURRENT_LIMIT_AMPS`)

Limits the current drawn from the battery per motor. Set this at or below your PDH breaker rating for the drive circuit.

- 40 A breaker: 40 A max
- 60 A breaker: 60 A max

### Tuning Process

1. Start with stator at `80.0` and supply at `60.0`.
2. Drive aggressively and watch for brownouts on the Driver Station.
3. If brownouts occur, lower supply current first.
4. If acceleration feels weak, raise stator current incrementally.
5. Recheck tipping behavior after any stator increase.

---

## Joystick Deadbands

Deadbands prevent the robot from creeping when the joystick is near center. Set in `RobotContainer.java`:

```java
new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.05)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
```

| Deadband | Value | What it means |
|---|---|---|
| Linear | 5% of max speed (~0.19 m/s) | Joystick must move past 5% before robot translates |
| Rotational | 10% of max angular rate (~0.47 rad/s) | Joystick must move past 10% before robot rotates |

Rotational deadband is intentionally larger than linear. Small accidental rotational inputs are more disruptive than small translational ones, so a wider deadband filters them out.

### When to change deadbands

- Robot creeps or twitches at rest: raise the deadband (try 0.08 linear, 0.12 rotational)
- Robot feels unresponsive off center: lower the deadband (try 0.03 linear, 0.07 rotational)
- Modules constantly realigning when stationary: raise rotational deadband

The deadband is applied by the CTRE SwerveRequest, not by joystick filtering. This means the full joystick range above the deadband is still mapped to full speed.

---

## Swerve Module Steer PID

These control how aggressively each swerve module snaps to its target angle. The gains live in `TunerConstants.java` under `steerGains`.

**Important:** TunerConstants is generated by Tuner X. If you regenerate, your changes will be overwritten. Note your tuned values somewhere before regenerating.

### Current values

| Gain | Value | Role |
|---|---|---|
| kP | 100 | Corrects angle error. Volts per rotation of error. |
| kI | 0 | Not used. |
| kD | 0.5 | Damps oscillation at the target angle. |
| kS | 0.1 | Overcomes friction to start turning the module. |
| kV | 2.66 | Main effort while the module is rotating. |
| kA | 0 | Not used. |

### The clanking / ratcheting sound at slow turns

A clanking or ratcheting sound during slow rotation is the modules oscillating around their target angle. At slow speeds, the target angle changes slowly so the module reaches its target, overshoots, corrects back, overshoots again. You hear each oscillation as a clank. At fast speeds, the target is constantly moving so the module never settles long enough to oscillate.

The cause is usually **steer kP being too aggressive**. Tuner X generates kP=100, which is a known starting point that works but can be harsh. There are Chief Delphi threads documenting teams having this exact issue with Tuner X default PIDs.

### Tuning steer kP

1. Start from the current value (100).
2. Lower to 80 and drive slowly in a circle. Listen for clanking.
3. If clanking is reduced, try 60. Keep lowering in steps of 10-20.
4. Stop when clanking disappears but modules still snap to angle quickly during fast maneuvers.
5. If modules feel sluggish during fast direction changes, you went too low. Go back up.

Typical range teams have landed on: 50-100 depending on the robot. Your 2025 bot ran 100 without issues, so the difference is likely mechanical (different module friction, weight distribution, or carpet surface).

### Tuning steer kD

If lowering kP alone doesn't fix the oscillation, raise kD. kD resists rapid angle changes, which dampens the overshoot that causes clanking.

- Current value: 0.5
- Try 1.0 to 2.0 if oscillation persists after lowering kP
- Too high: modules feel "sticky" and slow to respond to fast direction changes

### Tuning steer kS and kV

These are feedforward gains and generally don't need to change unless you swap module hardware.

- kS (0.1): voltage to overcome static friction. Raise if modules hesitate before turning.
- kV (2.66): voltage per RPS of target rotation speed. This is the main effort while steering. If modules lag behind during fast turns, raise it. If they overshoot past the target angle, lower it.

### Quick reference

| Symptom | What to change |
|---|---|
| Clanking at slow turns | Lower steer kP (try 80, then 60) |
| Modules oscillate visibly at rest | Lower steer kP or raise steer kD |
| Modules sluggish during fast maneuvers | Raise steer kP back up |
| Modules overshoot target angle | Raise steer kD |
| Modules hesitate before turning | Raise steer kS |

---

Team 8041 - Lake City Ultrabots, 2026
