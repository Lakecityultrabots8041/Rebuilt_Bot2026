# Shooter Tuning Guide

All tunable values live in: `src/main/java/frc/robot/subsystems/shoot/ShooterConstants.java`

---

## How the Mechanism Works

The shooter has three motors, split into two jobs:

**Feed rollers:** `actFloor` (ID 5) and `actCeiling` (ID 7) on CANivore "Jeffery".
These are the active floor and ceiling that push the ball from the intake up into the flywheel. They run on `DutyCycleOut`, which is simple power with no PID. They do not affect how far the ball goes. They just need enough force to push the ball in consistently.

**Flywheel:** `flywheelMotor` (ID 6) on CANivore "Jeffery".
This is what actually launches the ball. It uses `VelocityVoltage` (PID closed-loop) because the exit speed needs to be precise and consistent. Faster flywheel means more distance.

### Why feed rollers don't use PID

The feed rollers are under heavy, variable compression as the ball passes through. Velocity PID fights that load. It tries to maintain a target RPS but the mechanical resistance is constantly changing. That causes stalling, jerky behavior, and inconsistent feeding. `DutyCycleOut` just says "push at 80% power" regardless of load. If the ball compresses the rollers, they slow down and push harder. If load is light, they spin faster. Either way the ball moves through.

### Shooting sequence (how the code works)

1. Flywheel spins up to target RPS. Feed rollers are OFF during this.
2. Code waits until flywheel actually reaches that speed.
3. Feed rollers turn on and push the ball into the spinning flywheel.
4. Ball exits, everything idles.

This order matters. If the ball enters before the flywheel is at speed, the shot will be short and inconsistent.

---

## Constants Reference

All of these are in `ShooterConstants.java`:

### Feed Roller Power

| Constant | Default | What it controls |
|---|---|---|
| `FEED_POWER` | 0.80 | Power to run feed rollers when shooting (0.0 to 1.0) |
| `EJECT_POWER` | -0.60 | Power when ejecting a stuck ball (negative = reverse) |
| `PASS_POWER` | 0.60 | Power when passing |

### Flywheel Speed Presets

| Constant | Default | What it controls |
|---|---|---|
| `FLYWHEEL_READY_RPS` | 105.0 | Full shooting speed |
| `FLYWHEEL_REV_RPS` | 75.0 | Pre-spin speed before going to full power |
| `FLYWHEEL_PASS_RPS` | 65.0 | Passing speed |
| `FLYWHEEL_IDLE_RPS` | 0.0 | Stopped |

### Flywheel PID

| Constant | Default | Role |
|---|---|---|
| `FLYWHEEL_kP` | 3.0 | Proportional, corrects velocity error at speed |
| `FLYWHEEL_kV` | 0.15 | Velocity feedforward, main effort while spinning |
| `FLYWHEEL_kS` | 0.25 | Static feedforward, overcomes friction to start |

### Tolerances

| Constant | Default | What it controls |
|---|---|---|
| `FLYWHEEL_TOLERANCE_RPS` | 2.0 | How close flywheel must be to target to count as "ready" |
| `READY_TIMEOUT_SECONDS` | 3.0 | Gives up waiting for flywheel after this long |

---

## Step 1 - Tune FEED_POWER First

This is the easiest thing to tune and should be done before any flywheel work.

The goal: ball moves through the feed path smoothly and consistently without stalling or slamming.

1. With the robot stationary and flywheel OFF, command "Start Feeding"
2. Watch the ball travel through the feed path
3. If the ball stalls or barely moves, raise `FEED_POWER`
4. If the ball slams through violently or the rollers sound strained, lower `FEED_POWER`
5. Typical working range: 0.70 to 0.90

Do NOT try to fix a stalling feed roller by tuning PID. There is no PID on these motors. If the ball stalls at `FEED_POWER = 0.90`, the problem is mechanical. Check compression, obstructions, and belt tension.

### Signs FEED_POWER is wrong

| Symptom | Adjustment |
|---|---|
| Ball stalls halfway through | Raise FEED_POWER |
| Ball makes grinding or straining sound | Lower FEED_POWER |
| Ball gets stuck and motors stall | Check mechanical clearance first, then raise FEED_POWER |
| Ball shoots out erratically before hitting flywheel | Lower FEED_POWER |

---

## Step 2 - Tune Flywheel PID

The flywheel needs to hit its target RPS reliably. Tune these gains in order.

### kS: Static friction override

Set kP and kV to 0 first. Increase kS from 0 until the flywheel reliably starts spinning from rest when commanded. If you command 105 RPS and the flywheel takes more than half a second to even start moving, kS is too low.

Too high: the flywheel lurches or spikes on startup.
Typical range: 0.1 to 0.5

### kV: Velocity feedforward

Keep kP at 0. Increase kV until `Flywheel/Actual RPS` on SmartDashboard gets close to `Flywheel/Target RPS`. This is the main tuning knob. It tells the motor how much voltage to apply per RPS of target speed.

Too low: flywheel runs consistently below target.
Too high: flywheel runs consistently above target.
Typical range: 0.10 to 0.20

### kP: Proportional correction

Re-enable kP. Increase from 0 until the flywheel closes the remaining gap between actual and target RPS. This handles the last 3 to 5 RPS of error that kV leaves behind.

Too high: `Flywheel/Actual RPS` oscillates up and down around the target.
Too low: flywheel sits a few RPS below target permanently.
Typical range: 1.0 to 5.0

### SmartDashboard values to watch

| Key | What to look for |
|---|---|
| `Flywheel/Actual RPS` | Should match Target RPS within 2 RPS at steady state |
| `Flywheel/Target RPS` | Should show the preset value for the current state |
| `Flywheel/Current` | Should drop after spin-up. Sustained current over 60A means gains are too aggressive. |
| `Shooter/Flywheel Ready` | Should go true within 2 to 3 seconds of commanding |

---

## Step 3 - Tune Shot Distance and Arc

Once the flywheel reaches its target speed consistently, tune the speed presets for the right shot distance.

### FLYWHEEL_READY_RPS: normal scoring shot

1. Position robot at your expected shooting distance
2. Command shoot sequence, watch ball exit
3. Shot too short, raise `FLYWHEEL_READY_RPS`
4. Shot too long, lower `FLYWHEEL_READY_RPS`

Start with a fixed position before trying to tune distance-based shots.

### FLYWHEEL_SPEED_RATIO: vision tracking (Limelight auto-aim)

In auto-aim mode, flywheel speed is calculated from the distance table and the ratio controls how the flywheel scales. This ratio exists for vision tracking only. Fixed presets use their own constants.

If shots in auto-aim mode are consistently short at all distances, raise ratio slightly (try 1.20).
If shots are consistently long, lower ratio (try 1.10).

### Distance table: auto-aim interpolation

```java
private static final double[] DISTANCE_TABLE_METERS = {1.5, 2.0, 2.46, 3.0, 3.5, 4.0, 5.0};
private static final double[] VELOCITY_TABLE_RPS    = { 60,  70,   80,  85,  88,  90,  90};
```

This table maps shooting distance (from Limelight) to flywheel RPS. The code interpolates between rows automatically. You don't need to add every possible distance, just enough points to cover your shooting range.

How to tune the table:
1. Enable auto-aim (left bumper)
2. Stand at each distance listed in the table
3. Watch `Flywheel/Actual RPS` on SmartDashboard to confirm the right velocity is being commanded
4. Shoot and note if ball lands short or long at that distance
5. Adjust the corresponding RPS in `VELOCITY_TABLE_RPS`

If you never shoot from certain distances, you can remove those rows. The table only needs to cover the real shooting positions used in a match.

---

## Step 4 - Tune Passing

Passing uses `FLYWHEEL_PASS_RPS` for the flywheel and `PASS_POWER` for the feed rollers. Feed power during a pass is lower than during a shot because you are not slamming the ball into a high-speed flywheel.

| Constant | Default | Direction |
|---|---|---|
| `FLYWHEEL_PASS_RPS` | 65.0 | Up for more range, down if overshooting |
| `PASS_POWER` | 0.60 | Up if ball stalls during a pass, down if too aggressive |

Passing is lower precision than shooting, so these don't need to be tuned as tightly.

---

## Step 5 - Eject Tuning

The eject command runs feed rollers in reverse to clear a stuck ball. Flywheel stays idle during eject.

`EJECT_POWER = -0.60`. If the ball doesn't clear, raise the magnitude (try -0.70). If it's too violent, lower it.

---

## Sequences Reference

### ShootSequence (used in autonomous)

```
readyFlywheel()          -> flywheel to FLYWHEEL_READY_RPS
waitUntilFlywheelReady() -> wait up to READY_TIMEOUT_SECONDS
startFeeding()           -> feed rollers on at FEED_POWER
wait 0.5 seconds         -> hold while ball fires
idleAll()                -> everything off
```

### Teleop Trigger (right trigger)

```
shoot()  -> flywheel + feed both start simultaneously
idle()   -> called when trigger is released
```

Note: The trigger starts both at once with no wait. If the driver holds the trigger before the flywheel is at speed, the shot may be short. Have drivers pre-rev with auto-aim before pulling the trigger for important shots.

### QuickShoot

Assumes flywheel is already at speed. Skips the wait and just runs the feed. Used in `smartShoot()` when `isFlywheelReady()` returns true.

---

## Named Commands (PathPlanner)

| Name | What it does |
|---|---|
| `Rev Shooter` | Spins flywheel to FLYWHEEL_REV_RPS (pre-spin) |
| `Shoot` | Flywheel to READY and feed on simultaneously |
| `Idle Shooter` | Everything off |
| `Pass` | Flywheel and feed to pass speeds simultaneously |
| `AlignAndShoot` | Limelight hub align, then full shoot sequence |

---

## Pre-Match Checklist

- [ ] `Flywheel/Actual RPS` responds when shooter is commanded
- [ ] `Shooter/Flywheel Ready` goes true within 3 seconds
- [ ] Feed rollers push ball through cleanly at FEED_POWER
- [ ] ShootSequence fires ball consistently at expected scoring distance
- [ ] Eject clears a stuck ball when commanded
- [ ] Auto-aim (left bumper) spins flywheel to distance-correct speed
- [ ] `Flywheel/Current` is not excessively high at steady state

---

## Quick Reference: What to Change for Each Problem

| Problem | What to change |
|---|---|
| Ball stalls in feed path | Raise `FEED_POWER` |
| Feed rollers sound like they're straining | Lower `FEED_POWER` or check mechanical compression |
| Shot too short (fixed position) | Raise `FLYWHEEL_READY_RPS` |
| Shot too long (fixed position) | Lower `FLYWHEEL_READY_RPS` |
| Shot too short at one auto-aim distance | Raise that distance's RPS in `VELOCITY_TABLE_RPS` |
| Flywheel runs below target consistently | Raise `FLYWHEEL_kV` |
| Flywheel oscillates around target | Lower `FLYWHEEL_kP` |
| Flywheel slow to start from rest | Raise `FLYWHEEL_kS` |
| Flywheel never hits "ready" in time | Raise `READY_TIMEOUT_SECONDS` or improve kV and kP |
| Teleop shots inconsistent | Drivers need to pre-rev before pulling trigger |
| Ball fired before flywheel at speed in auto | `READY_TIMEOUT_SECONDS` may have expired, check flywheel PID |
| Eject doesn't clear ball | Raise `EJECT_POWER` magnitude (try -0.70) |
| Pass too short | Raise `FLYWHEEL_PASS_RPS` |
| Pass too long | Lower `FLYWHEEL_PASS_RPS` |

---

Team 8041 - Lake City Ultrabots, 2026
