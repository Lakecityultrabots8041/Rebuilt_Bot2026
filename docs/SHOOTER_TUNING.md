# Shooter Tuning Guide

All tunable values live in: `src/main/java/frc/robot/subsystems/shoot/ShooterConstants.java`

---

## How the Mechanism Works

The shooter has five motors, split into two jobs:

**Feed rollers:** `feedFloor` (CAN 5), `feedCeiling` (CAN 7), and `lo4der` (CAN 8).
Floor and ceiling are direct drive and push the ball horizontally toward the flywheel. Lo4d3r has a 12:1 gearbox and 4 belts that pull the ball up into the flywheel from above. All three use DutyCycleOut (percent power, no PID). They don't affect shot distance, they just move the ball.

**Flywheel:** `flywheelMotor` (CAN 6) and `flywheelMotor2` (CAN 9).
These launch the ball. They use VelocityVoltage (PID closed-loop) because exit speed needs to be the same every shot. Faster flywheel = more distance. The hood is fixed, so flywheel speed is the only way to control distance.

### Why feed rollers use percent power instead of PID

Feed rollers are under heavy, variable compression as the ball passes through. Velocity PID fights that load, causing stalling and jerky feeding. DutyCycleOut just says "push at 80% power" regardless of load. If the ball compresses the rollers, they slow down and push harder. If load is light, they spin faster. Either way the ball moves through.

### Why the flywheel uses PID

A shot from 8 feet needs a different exit speed than a shot from 13 feet. DutyCycleOut changes with battery voltage (80% of 12.5V is different from 80% of 11.0V), so shots would be inconsistent as the battery drains. VelocityVoltage compensates automatically and holds the exact RPS you ask for.

### Shooting sequence (how the code works)

1. Flywheel spins up to target RPS. Feed rollers are OFF during this.
2. Code waits until flywheel actually reaches that speed (checks real velocity, not a timer).
3. Feed rollers turn on and push the ball into the spinning flywheel.
4. Ball exits, everything idles.

If the ball enters before the flywheel is at speed, the shot will be short.

### Why not use a timer to wait?

An early version (testDelayedShot) blindly waited 1.5 seconds instead of checking velocity. That fires short if the flywheel is slow, or wastes time if it reaches speed early. shootSequence() in ShooterCommands.java checks real velocity against FLYWHEEL_TOLERANCE_RPS before feeding. Faster and more reliable.

---

## Constants Reference

### Feed Roller Power

| Constant     | Value | What it does                                  |
|--------------|-------|-----------------------------------------------|
| `FEED_POWER` | 0.80  | Power when shooting (pushes ball to flywheel) |
| `EJECT_POWER`| -0.20 | Power when ejecting (negative = reverse)      |
| `PASS_POWER` | 0.80  | Power when passing                            |

### Lo4d3r Power

| Constant             | Value | What it does                                  |
|----------------------|-------|-----------------------------------------------|
| `LO4D3R_POWER`      | -0.80 | Power when feeding (pulls ball up)            |
| `LO4D3R_EJECT_POWER`| 0.50  | Power when ejecting (pushes ball back down)   |

### Flywheel Speed Presets

| Constant             | Value | What it does                              |
|----------------------|-------|-------------------------------------------|
| `FLYWHEEL_READY_RPS` | 43.0  | Default shooting speed (fixed mode)       |
| `FLYWHEEL_REV_RPS`   | 30.0  | Pre-spin speed before going to full power |
| `FLYWHEEL_PASS_RPS`  | 50.0  | Passing speed                             |
| `FLYWHEEL_IDLE_RPS`  | 0.0   | Stopped                                   |

### Flywheel PID

| Constant      | Value | What it does                                     |
|---------------|-------|--------------------------------------------------|
| `FLYWHEEL_kP` | 3.0   | Corrects remaining velocity error once spinning  |
| `FLYWHEEL_kV` | 0.15  | Main effort, volts per RPS of target speed       |
| `FLYWHEEL_kS` | 0.25  | Overcomes friction to start the motor moving     |

### Tolerances

| Constant                 | Value | What it does                                         |
|--------------------------|-------|------------------------------------------------------|
| `FLYWHEEL_TOLERANCE_RPS` | 2.0   | How close flywheel must be to target to count as ready |
| `READY_TIMEOUT_SECONDS`  | 3.0   | Gives up waiting for flywheel after this long         |

### Supply Current Limits

Supply limits prevent brownouts. No stator limits (254 style).

| Constant                       | Value | What motor                            |
|--------------------------------|-------|---------------------------------------|
| `FLYWHEEL_SUPPLY_CURRENT_LIMIT`| 45.0 A| Flywheel (CAN 6 + 9)                 |
| `FEED_SUPPLY_CURRENT_LIMIT`    | 60.0 A| Feed floor + ceiling (CAN 5 + 7)     |
| `LO4D3R_SUPPLY_CURRENT_LIMIT`  | 60.0 A| Lo4d3r upper roller (CAN 8)          |

---

## Step 1 - Tune FEED_POWER First

The goal: ball moves through the feed path smoothly without stalling or slamming.

1. With the robot stationary and flywheel OFF, command "Start Feeding"
2. Watch the ball travel through the feed path
3. If the ball stalls or barely moves, raise `FEED_POWER`
4. If the ball slams through violently or the rollers sound strained, lower `FEED_POWER`
5. Typical range: 0.70 to 1.0

Do NOT try to fix a stalling feed roller by tuning PID. There is no PID on these motors. If the ball stalls at full power, the problem is mechanical (compression, obstructions, belt tension).

| Symptom                                             | Fix                                                     |
|-----------------------------------------------------|---------------------------------------------------------|
| Ball stalls halfway through                         | Raise FEED_POWER                                        |
| Ball makes grinding or straining sound              | Lower FEED_POWER                                        |
| Ball gets stuck and motors stall                    | Check mechanical clearance first, then raise FEED_POWER |
| Ball shoots out erratically before hitting flywheel | Lower FEED_POWER                                        |

---

## Step 2 - Tune Flywheel PID

Tune these gains in order: kS first, then kV, then kP.

### kS: Static friction override

Set kP and kV to 0 first. Increase kS from 0 until the flywheel reliably starts spinning from rest. If the flywheel takes more than half a second to start moving, kS is too low.

Too high: flywheel lurches on startup.
Typical range: 0.1 to 0.5

How to measure: slowly increase voltage from 0 on a free-spinning flywheel. The voltage where the motor just barely starts to move is your kS.

### kV: Velocity feedforward

Keep kP at 0. Increase kV until Flywheel/Actual RPS gets close to Flywheel/Target RPS on SmartDashboard. This is the main tuning knob.

Too low: flywheel runs consistently below target.
Too high: flywheel runs consistently above target.
Typical range: 0.10 to 0.20

How to calculate a starting value: `kV = 12V / free speed in RPS`. For a Kraken X60 with no gear reduction: `12 / 100 = 0.12`. Our 0.15 was tuned on the robot.

### kP: Proportional correction

Re-enable kP. Increase from 0 until the flywheel closes the remaining gap between actual and target.

Too high: Flywheel/Actual RPS oscillates up and down around target.
Too low: flywheel sits a few RPS below target permanently.
Typical range: 1.0 to 5.0

### How the PID math works

Every loop, Phoenix 6 calculates:

```
Output Voltage = kS * sign(target) + kV * target_RPS + kP * error_RPS
```

Where `error_RPS = target_RPS - actual_RPS`.

- **kS (0.25V):** Flat voltage to overcome friction. Always applied when motor should be spinning.
- **kV (0.15 V/RPS):** Main effort. At 100 RPS target, kV contributes `0.15 * 100 = 15.0V`.
- **kP (3.0 V/RPS error):** Correction. If 2 RPS below target, kP adds `3.0 * 2 = 6.0V`.

| Gain | Unit        | Meaning                                 |
|------|-------------|-----------------------------------------|
| kP   | Volts / RPS | How hard to push per RPS of error       |
| kV   | Volts / RPS | How much base voltage per RPS of target |
| kS   | Volts       | Flat voltage to overcome friction       |

### SmartDashboard values to watch

| Key                        | What to look for                                                  |
|----------------------------|-------------------------------------------------------------------|
| `Flywheel/Actual RPS`     | Should match Target RPS within 2 RPS at steady state              |
| `Flywheel/Target RPS`     | Should show the preset value for the current state                |
| `Flywheel/Current`        | Should drop after spin-up. Sustained > 60A means gains are wrong  |
| `Shooter/Flywheel Ready`  | Should go true within 2-3 seconds of commanding                   |

---

## Step 3 - Tune Shot Distance

Once the flywheel reaches its target speed consistently, tune the speed presets.

### Fixed mode (FLYWHEEL_READY_RPS)

1. Position robot at your expected shooting distance
2. Command shoot, watch ball exit
3. Shot too short: raise `FLYWHEEL_READY_RPS`
4. Shot too long: lower `FLYWHEEL_READY_RPS`

### Distance mode (distance table)

When distance mode is on (back button toggle), the flywheel picks its speed from this table based on Limelight distance:

```java
DISTANCE_TABLE_FEET = { 5.0,  6.5,   8.0,  10.0, 11.5, 13.0, 16.5}
VELOCITY_TABLE_RPS  = {  35,   40,    45,    50,   55,   58,   60}
```

The code interpolates between entries. You don't need every possible distance, just enough points to cover your shooting range. The table is in feet for easier tuning. Limelight gives meters, the code converts automatically.

How to tune:
1. Turn on distance mode (back button)
2. Stand at each distance in the table
3. Watch `Flywheel/Target RPS` to confirm the right speed is being commanded
4. Shoot and note if ball lands short or long
5. Adjust the corresponding RPS in `VELOCITY_TABLE_RPS`

If distance mode doesn't work before comp, hit the back button to switch to fixed mode (uses FLYWHEEL_READY_RPS for every shot).

---

## Step 4 - Tune Passing

| Constant            | Value | Adjust                                          |
|---------------------|-------|-------------------------------------------------|
| `FLYWHEEL_PASS_RPS` | 50.0  | Up for more range, down if overshooting         |
| `PASS_POWER`        | 0.80  | Up if ball stalls during a pass, down if too aggressive |

Passing is lower precision than shooting, doesn't need tight tuning.

---

## Step 5 - Tune Eject

Eject runs feed rollers in reverse to clear a stuck ball. Flywheel stays idle.

`EJECT_POWER = -0.20`. If the ball doesn't clear, raise the magnitude (try -0.50 or -0.80). If it's too violent, lower it.

---

## Sequences Reference

### shootSequence (used in autonomous)

```
readyFlywheel()          -> flywheel to FLYWHEEL_READY_RPS (43 RPS)
waitUntilFlywheelReady() -> wait up to READY_TIMEOUT_SECONDS (3s)
startFeeding()           -> feed rollers on at FEED_POWER
startLo4d()              -> lo4d3r on at LO4D3R_POWER
wait 0.5 seconds         -> hold while ball fires
idleAll()                -> everything off
```

### Teleop trigger (right trigger)

When distance mode is ON (back button):
```
shootWithDistance() -> reads Limelight distance each loop, picks RPS from table
                   -> falls back to 43 RPS if no target visible
idle()             -> called when trigger is released
```

When distance mode is OFF:
```
shootWithDistance() -> fixed 43 RPS (FLYWHEEL_READY_RPS)
idle()             -> called when trigger is released
```

### quickShoot

Skips the flywheel wait. Only safe when flywheel is already at speed. Used by smartShoot() when isFlywheelReady() returns true.

### smartShoot

Checks if flywheel is already spinning. If yes, just feeds (quickShoot). If no, does the full shootSequence.

---

## Named Commands (PathPlanner)

| Name                    | What it does                                          |
|-------------------------|-------------------------------------------------------|
| `Rev Flywheel`          | Spins flywheel to FLYWHEEL_REV_RPS (30 RPS pre-spin) |
| `Shoot`                 | Flywheel to READY and feed on simultaneously          |
| `Quick Shoot`           | Same as Shoot but used for faster auto sequences      |
| `Idle Shooter`          | Everything off                                        |
| `Pass`                  | Flywheel and feed to pass speeds                      |
| `AlignHubAndShoot`      | Hub align, then quick shoot                           |
| `AlignOutpostAndShoot`  | Outpost align, then shoot sequence                    |
| `AlignTowerAndShoot`    | Tower align, then shoot sequence                      |

---

## Pre-Match Checklist

- [ ] `Flywheel/Actual RPS` responds when shooter is commanded
- [ ] `Shooter/Flywheel Ready` goes true within 3 seconds
- [ ] Feed rollers push ball through cleanly at FEED_POWER
- [ ] shootSequence fires ball consistently at expected distance
- [ ] Eject clears a stuck ball when commanded
- [ ] Distance mode (back button) shows correct RPS for different distances
- [ ] `Flywheel/Current` is not excessively high at steady state

---

## Quick Reference: What to Change for Each Problem

| Problem                                  | What to change                                    |
|------------------------------------------|---------------------------------------------------|
| Ball stalls in feed path                 | Raise `FEED_POWER`                                |
| Feed rollers sound strained              | Lower `FEED_POWER` or check mechanical compression |
| Shot too short (fixed mode)              | Raise `FLYWHEEL_READY_RPS`                        |
| Shot too long (fixed mode)               | Lower `FLYWHEEL_READY_RPS`                        |
| Shot wrong at one distance               | Adjust that row in `VELOCITY_TABLE_RPS`           |
| Flywheel runs below target consistently  | Raise `FLYWHEEL_kV`                               |
| Flywheel oscillates around target        | Lower `FLYWHEEL_kP`                               |
| Flywheel slow to start from rest         | Raise `FLYWHEEL_kS`                               |
| Flywheel never hits ready in time        | Raise `READY_TIMEOUT_SECONDS` or improve kV/kP    |
| Teleop shots inconsistent               | Check distance mode, or pre-align before shooting  |
| Ball fired before flywheel at speed      | `READY_TIMEOUT_SECONDS` may have expired           |
| Eject doesn't clear ball                | Raise `EJECT_POWER` magnitude (try -0.50)          |
| Pass too short                          | Raise `FLYWHEEL_PASS_RPS`                          |
| Pass too long                           | Lower `FLYWHEEL_PASS_RPS`                          |

---

Team 8041 - Lake City Ultrabots, 2026
