# Shooter Tuning Guide

All tunable values live in: `src/main/java/frc/robot/subsystems/shoot/ShooterConstants.java`

---

## How the Mechanism Works

The shooter has two motors: **shooter** (bottom wheel, motor ID 5) and **flywheel** (top wheel, motor ID 6). Fuel passes between them. Both wheels spin in directions that push the fuel out.

The difference in speed between the two wheels is what puts spin on the fuel, and spin is what creates the arc.

- Both wheels same speed: minimal spin, flatter more direct exit
- Flywheel faster than shooter: backspin on the fuel, the shot arcs upward and carries farther
- Bigger ratio: more backspin, higher arc
- Higher absolute speed (both wheels faster): more velocity, longer range at any given arc

`FLYWHEEL_SPEED_RATIO` in ShooterConstants controls how much faster the flywheel runs relative to the shooter in auto-aim mode. The preset velocities (`FLYWHEEL_MAX_VELOCITY` vs `MAX_VELOCITY`) control the same thing for button-triggered shots.

---

## Constants Reference

All of these are in `ShooterConstants.java`:

| Constant | Default | What it controls |
|---|---|---|
| `MAX_VELOCITY` | 90 RPS | Shooter motor speed for normal scoring shots |
| `FLYWHEEL_MAX_VELOCITY` | 105 RPS | Flywheel speed for normal scoring shots |
| `FLYWHEEL_SPEED_RATIO` | 1.15 | Flywheel multiplier in auto-aim mode (flywheel = shooter x ratio) |
| `PASS_VELOCITY` | 100 RPS | Shooter speed for long-distance passing |
| `FLYWHEEL_PASS_VELOCITY` | 110 RPS | Flywheel speed for long-distance passing |
| `VELOCITY_TOLERANCE_RPS` | 2.0 RPS | How close motors must be to target before "ready" |
| `READY_TIMEOUT_SECONDS` | 2.0 s | Max wait for motors to reach speed |
| `kP / kV / kS` | 3.0 / 0.15 / 0.25 | Shooter motor PID (see below) |
| `FLYWHEEL_kP / kV / kS` | 3.0 / 0.15 / 0.25 | Flywheel motor PID (see below) |

---

## Tuning the Arc (Shot Shape)

### Step 1 - Get the right exit velocity first

Before worrying about arc, get the shot reaching the hub consistently.

1. Start with `MAX_VELOCITY = 90` and `FLYWHEEL_MAX_VELOCITY = 105`.
2. Watch `Shooter/Actual RPS` and `Flywheel/Actual RPS` on SmartDashboard.
3. If the shot falls short, increase both velocities proportionally.
4. If the shot overshoots, decrease both.

### Step 2 - Tune the arc shape

Once distance is right, adjust the ratio to get the arc you want.

- Shot too flat or hits the rim low: increase `FLYWHEEL_MAX_VELOCITY` (or increase `FLYWHEEL_SPEED_RATIO` for auto-aim shots)
- Shot too high or arcs over the target: decrease `FLYWHEEL_MAX_VELOCITY`
- Shot not consistent: check PIDs first (section below), then revisit velocities

A 10-20% flywheel advantage is a typical starting range. Current defaults are about 16.7% (105 / 90).

### Step 3 - Distance-based velocity (auto-aim mode)

In auto-aim, shooter velocity is interpolated from the distance table and flywheel = shooter x `FLYWHEEL_SPEED_RATIO`. The table is in ShooterConstants:

```java
private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
private static final double[] VELOCITY_TABLE_RPS    = { 60,  70,   80,  85,  88,  90,  90};
```

Tune these on the real field. Shoot from each distance, note whether the shot is long or short, and adjust the corresponding RPS value. The code interpolates between rows automatically.

The flywheel follows the same ratio at every distance. If you find the arc changes too much at different ranges, `FLYWHEEL_SPEED_RATIO` can be adjusted to fix it globally.

---

## PID Tuning (VelocityTorqueCurrentFOC)

The shooter uses `VelocityTorqueCurrentFOC`, which is torque-current based closed-loop velocity control. Three gains matter:

| Gain | Role | Units | Tune when |
|---|---|---|---|
| `kS` | Static feedforward, overcomes friction to get the motor moving | Amps | Motor hesitates to start or has a dead band |
| `kV` | Velocity feedforward, main effort at speed | Amps per RPS | Motor runs consistently above or below target |
| `kP` | Proportional feedback on velocity error | Amps per RPS error | Motor is close but not quite hitting target |

### Tuning order: kS first, then kV, then kP

**kS** - Start at 0. Increase until the motor reliably starts spinning from rest without hesitation. Too high and the motor jerks on startup. Typical: 0.1-0.5 A.

**kV** - Set kP = 0 first. Increase kV until the motor runs close to the target RPS at steady state. Watch `Shooter/Actual RPS` vs `Shooter/Commanded RPS` on SmartDashboard. Too low and the motor runs slow, too high and it runs fast. Typical: 0.1-0.2 A/RPS.

**kP** - Re-enable kP. Increase from 0 until steady-state error closes. Too high causes oscillation where the RPS bounces around the target. Too low and the motor sits 3-5 RPS below target. Typical: 1.0-5.0.

The flywheel has its own independent gains (`FLYWHEEL_kP`, `FLYWHEEL_kV`, `FLYWHEEL_kS`). Start identical to the shooter gains, then tune separately if the flywheel has different mechanical characteristics.

### SmartDashboard values to watch during PID tuning

| Key | What to look for |
|---|---|
| `Shooter/Actual RPS` | Should match Commanded RPS within 2 RPS |
| `Shooter/Commanded RPS` | Should be the preset value for the current state |
| `Shooter/At Target` | Should go true within 1-2 seconds of commanding |
| `Flywheel/Actual RPS` | Same check for flywheel |
| `Shooter/Motor Current` | Excessive current (over 60A sustained) means PID is too aggressive or gains are too high |

---

## Passing Shot Setup

The passing command (B button) spins up to `PASS_VELOCITY` / `FLYWHEEL_PASS_VELOCITY` instead of the normal scoring presets. These are intentionally set lower than scoring maximums because for a long-distance pass you typically want a flatter faster trajectory with less arc and more range.

### Constants to tune for passing

| Constant | Default | Direction to tune |
|---|---|---|
| `PASS_VELOCITY` | 100 RPS | Up for more range, down if overshooting |
| `FLYWHEEL_PASS_VELOCITY` | 110 RPS | Lower ratio than scoring = flatter arc for distance |

The default pass ratio is 110/100 = 1.10, intentionally less than the scoring ratio of 1.15. If you want a more lofted pass, increase `FLYWHEEL_PASS_VELOCITY`. If you want flatter, decrease it toward or below `PASS_VELOCITY`.

### Passing workflow in teleop

1. Drive toward the center of the field.
2. Hold **B button**, robot spins up to pass speed. SmartDashboard `Shooter/At Target` goes true.
3. Release **B button**, shooter idles.

---

## Tuning Checklist

### Initial setup (first time on robot)
- [ ] Verify `Shooter/Actual RPS` responds when shooter is commanded
- [ ] Verify `Flywheel/Actual RPS` responds independently of shooter
- [ ] Confirm `Shooter/At Target` goes true reliably within `READY_TIMEOUT_SECONDS`
- [ ] Tune kS, kV, kP for shooter motor
- [ ] Tune FLYWHEEL_kS, kV, kP separately if flywheel lags or oscillates

### Shot tuning
- [ ] Scoring shot reaches hub at 2.46m (default hub distance)
- [ ] Arc is consistent, not low and skipping off rim, not high and arcing over
- [ ] Adjust `FLYWHEEL_MAX_VELOCITY` for arc shape
- [ ] Walk through each distance in the table and verify auto-aim shots land correctly

### Passing tuning
- [ ] Pass shot reaches target from center field
- [ ] Trajectory is flat enough to not hit field elements
- [ ] `PASS_VELOCITY` and `FLYWHEEL_PASS_VELOCITY` confirmed on carpet

---

## Quick Reference - What to Change for Each Problem

| Problem | Change |
|---|---|
| Shot too short | Increase `MAX_VELOCITY` and `FLYWHEEL_MAX_VELOCITY` proportionally |
| Shot too long | Decrease both velocities |
| Shot arcs too high | Decrease `FLYWHEEL_MAX_VELOCITY` |
| Shot arcs too low or hits rim | Increase `FLYWHEEL_MAX_VELOCITY` |
| Auto-aim shots inconsistent arc | Adjust `FLYWHEEL_SPEED_RATIO` |
| Motor not reaching target speed | Tune kV upward, check kP isn't too high causing oscillation |
| Motor oscillates around target | Lower kP |
| Motor slow to start from rest | Raise kS |
| Pass too short | Increase `PASS_VELOCITY` |
| Pass too flat or no arc | Increase `FLYWHEEL_PASS_VELOCITY` |
| Pass too arced | Decrease `FLYWHEEL_PASS_VELOCITY` toward `PASS_VELOCITY` |

---

Team 8041 - Lake City Ultrabots, 2026
