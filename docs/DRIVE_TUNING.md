# Drive Tuning Guide

All tunable values live in one file: `src/main/java/frc/robot/subsystems/drive/DriveConstants.java`

---

## Asymmetric Slew Rate Limiting

The robot uses asymmetric slew rate limiting on the two translation axes (forward/back and strafe). Acceleration and deceleration are limited independently, so you can have a fast snappy feel without violent stops that tip the robot.

- `MAX_TELEOP_ACCEL` - how fast the commanded speed ramps up
- `MAX_TELEOP_DECEL` - how fast the commanded speed ramps down

Tune acceleration freely. Tune deceleration carefully. Too high of a decel value is what causes tipping on a top-heavy robot.

### Timing Reference at Full Speed (4.58 m/s)

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
