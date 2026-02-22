# Intake Tuning Guide

All tunable values live in: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`

---

## How the Mechanism Works

The intake has two motors with completely different jobs:

**Pivot motor** (ID 2, CANivore "Jeffery"): the arm that swings out and back.
Uses Motion Magic, which is a position controller that follows a smooth speed profile. You tell it where to go and it figures out how to get there at a controlled rate. This motor needs PID because it has to hit specific positions accurately.

**Intake motor** (ID 3, CANivore "Jeffery"): the roller that pulls balls in.
Uses `DutyCycleOut`, which is simple power with no PID. It just needs enough force to grab and pull a ball regardless of how hard the ball is pressing against it. Velocity PID fights variable load. Power control adapts to it.

### The arm on this robot

The arm is short, just long enough to reach past the bumpers to the balls. It is heavier than typical due to aluminum plates, compliant wheels, and backer bars. The backer bar physically rests on the bumper when the arm is at intake position. This means:

- At `INTAKE_POSITION`, the bumper is taking the arm's weight. The motor is not fighting gravity there.
- The hardest moment for the motor is stowing (lifting the arm back up against gravity).
- `kG` needs to be tuned for the weight of this specific arm and may need to be higher than a lighter arm would need.

### Three arm positions

| State | What it means |
|---|---|
| `STOW` | Arm inside the robot frame, legal starting position |
| `INTAKE` | Arm extended out over the bumper, backer bar resting on bumper, roller at ball height |
| `TRAVEL` | Arm held partway up, used as a safe position between intake and stow |

### On power-on

The code sets the pivot encoder to `STOW_POSITION = 0.0` and immediately commands the arm to hold stow. The arm must be physically at stow before powering on every time. If the arm is at any other position on boot, the encoder will be wrong and all positions will be off.

---

## First Time Setup - Verify Before Tuning Anything

1. Power on with the arm physically at stow (fully inside the robot)
2. Open SmartDashboard and confirm `Intake/Pivot Position` reads close to `0.0`
3. Press DPad Down (pivot to intake). The arm should swing out toward the ground.
4. Watch `Intake/Pivot Position` increase as the arm moves.
5. When the arm rests on the bumper, read the position value. That is close to your `INTAKE_POSITION`.
6. Press DPad Up (pivot to stow). The arm should swing back in.
7. Confirm it returns to about 0.0 and stops cleanly.

If the arm moves the wrong direction when commanded, set `pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive` in the pivot config block inside `IntakeSubsystems.java`.

If the arm drifts after stopping, brake mode may not have applied. Confirm motor ID matches physical hardware and the CAN bus name is "Jeffery".

If position reads something other than 0.0 on boot, the arm was not at stow when powered on. Power off, physically place arm at stow, power back on.

---

## Step 1 - Measure Real Position Values

The defaults in IntakeConstants are placeholders. You must measure the real values on the robot.

| Constant | Default | What it is |
|---|---|---|
| `STOW_POSITION` | 0.0 | Always 0.0. The encoder is seeded to this on boot. |
| `INTAKE_POSITION` | 5.0 | Encoder reading when arm is resting on bumper at intake height |
| `TRAVEL_POSITION` | 3.5 | Encoder reading at the safe in-between position |

### How to measure INTAKE_POSITION

1. Power on with arm at stow
2. Press DPad Down to command intake position
3. Watch `Intake/Pivot Position` on SmartDashboard
4. Wait until arm rests on the bumper and stops moving
5. Read the position and put that number in `INTAKE_POSITION`

### How to measure TRAVEL_POSITION

This is a judgment call. TRAVEL should be high enough that the arm clears the bumper when driving but low enough that it drops to intake quickly. A good starting point is halfway between STOW and INTAKE. Move the arm manually to the right height with the robot disabled in Phoenix Tuner, read the position, enter it.

### Update SOFT_LIMIT_FORWARD after measuring

Once you have the real `INTAKE_POSITION`, set `SOFT_LIMIT_FORWARD` to about `INTAKE_POSITION + 0.5`. This lets Motion Magic settle at the target without fighting the limit, while still preventing the motor from grinding into the bumper hard stop indefinitely. If the arm is pressing hard into the bumper during intake, lower this value.

---

## Step 2 - Tune Motion Magic (Arm Speed)

Motion Magic controls how fast and how smoothly the arm moves. These three values shape the motion profile:

| Constant | Default | What it controls |
|---|---|---|
| `CRUISE_VELOCITY` | 15 | Max speed during travel (rotations/sec) |
| `ACCELERATION` | 20 | How fast it ramps up to cruise speed (rot/sec2) |
| `JERK` | 20 | How smoothly acceleration changes. Lower means silkier start and stop. |

You want the arm to deploy fast. Tune in this order:

1. Start with defaults and command the arm to intake. Time how long it takes.
2. Raise `CRUISE_VELOCITY` to 25 and repeat. It should be noticeably faster.
3. Keep raising until it's as fast as you want without the arm slamming into the bumper.
4. If the arm slams or bounces at the target, that is a PID problem (see Step 3), not a Motion Magic problem.
5. Tune `ACCELERATION` last. Raising it makes the arm feel snappier off the start.

Watch `Intake/Pivot At Target` on SmartDashboard. It should go `true` after each move and stay `true`.

---

## Step 3 - Tune the Pivot PID and Feedforwards

These gains work together to hold the arm in position and follow the Motion Magic profile accurately.

| Gain | Default | Role |
|---|---|---|
| `kG` | 4.25 | Gravity compensation. Holds the arm up against gravity at any angle. |
| `kS` | 2.0 | Static friction. Overcomes the initial resistance before the arm starts moving. |
| `kV` | 4.5 | Velocity feedforward. Main effort while the arm is in motion. |
| `kA` | 0.01 | Acceleration feedforward. Helps during fast changes in speed. |
| `kP` | 12.0 | Proportional. Corrects remaining position error at the target. |
| `kI` | 0 | Integral. Leave at 0 unless arm consistently stops slightly short of target. |
| `kD` | 0.2 | Derivative. Damps oscillation at the target. |

### Tune in this order

**kG first, most important for a gravity arm**

This is what holds the arm up. Set kP, kV, and kS all to 0. Enable the robot. Raise kG until the arm holds its position without drifting when you hold it horizontal and let go.

Arm drifts down: raise kG.
Arm creeps up: lower kG.

`Arm_Cosine` is already set in the code, which means kG is automatically scaled by the cosine of the arm angle. You just find the right number and it handles the rest. At stow (arm vertical), gravity compensation is near zero. At horizontal, full kG is applied.

Important for this robot: this arm is heavier than a typical FRC intake arm due to the aluminum plates, compliant wheels, and backer bars. `kG = 4.25` is the starting point but you may need to go higher, potentially 5.0 to 6.5. Test by holding the arm horizontal (about the angle it is at during travel) and letting go. It should hold with minimal drift.

The arm resting on the bumper at intake position means kG matters most during the return to stow. That is the moment the motor is fighting the full weight of the arm being lifted. If the arm stalls or slows dramatically on the way back up to stow, kG is too low.

**kS second**

With kG set, command a move. If the arm hesitates before starting to move, raise kS slightly. You want the arm to begin moving immediately when commanded with no dead zone. Typical: 1.0 to 3.0.

**kV third**

If the arm moves slower than the Motion Magic profile expects during travel, raise kV. If it consistently runs ahead of the profile, lower it. Typical: 3.0 to 6.0.

**kP last**

With feedforwards set, the arm should arrive close to target on its own. kP handles the final position correction. Raise from 0 until the arm snaps cleanly to the target. Too high and the arm oscillates around the target. Too low and it stops a bit short.

**kD: damp oscillation**

If the arm bounces at the target after arriving, raise kD. It does not change how fast the arm moves, just how it settles. `kD = 0.2` is a reasonable starting point for a heavy arm.

### SmartDashboard values to watch

| Key | What to look for |
|---|---|
| `Intake/Pivot Position` | Should match the target position when arm is at rest |
| `Intake/Pivot At Target` | Should go `true` after each move and stay `true` |
| `Intake/Pivot State` | Confirms the state machine commanded the right position |

---

## Step 4 - Tune the Intake Roller

The roller runs on `DutyCycleOut`. There are no PID gains to tune. You only need to set the right power level.

| Constant | Default | What it is |
|---|---|---|
| `INTAKE_POWER` | 0.80 | Power when pulling balls in (0.0 to 1.0) |
| `EJECT_POWER` | -0.50 | Power when reversing to eject (negative = reverse) |

### How to tune INTAKE_POWER

1. Set arm to intake position (DPad Down)
2. Roll a ball in front of the intake roller
3. Command intake (DPad Left) and watch what happens
4. Ball gets pulled in cleanly: power is good
5. Ball slips off or barely moves: raise `INTAKE_POWER`
6. Roller makes straining sounds or ball gets jammed: lower `INTAKE_POWER`
7. Typical working range: 0.70 to 0.90

If the ball stalls at high power, the problem is mechanical. Check for obstructions, check belt tension, check that the compliant wheels have enough grip on the ball.

### How to tune EJECT_POWER

Command eject while a ball is in the intake. The ball should reverse out cleanly.
If it barely moves, raise the magnitude (try -0.60).
If it is too violent, lower the magnitude (try -0.40).

---

## Step 5 - Confirm Sequences Work End to End

The sequences are what autonomous uses. Test these in teleop before running any auto.

### Starting Intake Sequence (PathPlanner named command: "Start Intake")
1. Commands pivot to `INTAKE_POSITION`
2. Waits until pivot actually reaches the target (up to `PIVOT_TIMEOUT_SECONDS`)
3. Starts the intake roller

If the roller starts before the arm arrives, the pivot is too slow or `PIVOT_TIMEOUT_SECONDS` timed out. Check `Intake/Pivot At Target` on SmartDashboard. If it never goes `true`, the arm is not reaching the target.

If the pivot times out before reaching target, raise `PIVOT_TIMEOUT_SECONDS` first. Then investigate whether the arm physically reaches the position and whether the PID is holding it there.

### Ending Intake Sequence (PathPlanner named command: "End Intake")
1. Stops the roller immediately
2. Commands pivot to `TRAVEL_POSITION`

This should happen quickly. If the arm does not leave intake position, check that TRAVEL_POSITION is correctly measured and that Motion Magic is tuned.

---

## Current Limits

The pivot has current limits to protect the motor if the arm hits something.

| Constant | Default | What it does |
|---|---|---|
| `PIVOT_STATOR_CURRENT_LIMIT` | 60 A | Max torque the motor produces. Prevents grinding against the bumper. |
| `PIVOT_SUPPLY_CURRENT_LIMIT` | 35 A | Max current drawn from the battery. |

If the arm feels weak and stalls before reaching target, raise stator limit in small steps (try 70A).
If you see brownouts while moving the arm, lower supply limit.
If the arm slams into the hard stop and strains, the soft limit may be too high or stator limit is too high.

---

## Soft Limits

Soft limits stop the motor from commanding the arm past the physical range.

| Constant | Default | What it does |
|---|---|---|
| `SOFT_LIMIT_FORWARD` | 5.5 | Motor won't command past this toward intake. Set about 0.5 above INTAKE_POSITION. |
| `SOFT_LIMIT_REVERSE` | 0.0 | Motor won't command below this toward stow. Matches STOW_POSITION. |

After measuring the real `INTAKE_POSITION`, set `SOFT_LIMIT_FORWARD = INTAKE_POSITION + 0.5`. If you can hear the motor straining at the intake position, lower this value until the straining stops.

---

## Pre-Match Checklist

- [ ] Arm physically placed at stow before powering on
- [ ] `Intake/Pivot Position` reads near 0.0 on boot
- [ ] DPad Down deploys arm to intake position, arm rests on bumper
- [ ] DPad Up returns arm to stow cleanly
- [ ] `Intake/Pivot At Target` goes `true` at each position
- [ ] Arm lifts back to stow without stalling or slowing dramatically (kG check)
- [ ] Intake roller pulls balls in cleanly at `INTAKE_POWER`
- [ ] Eject reverses ball out cleanly
- [ ] "Start Intake" sequence runs fully. Roller does not start before arm arrives.
- [ ] "End Intake" sequence returns arm to travel
- [ ] No brownouts during any arm or roller operation

---

## Quick Reference: What to Change for Each Problem

| Problem | What to change |
|---|---|
| Arm moves wrong direction | Add or remove inversion in pivot config in `IntakeSubsystems.java` |
| Arm drifts down when holding position | Raise `kG` |
| Arm creeps up when holding position | Lower `kG` |
| Arm stalls or slows going back to stow | Raise `kG`. Motor is fighting gravity on a heavy arm. |
| Arm hesitates before starting to move | Raise `kS` |
| Arm too slow deploying | Raise `CRUISE_VELOCITY` |
| Arm overshoots and bounces at target | Lower `kP` or lower `CRUISE_VELOCITY` |
| Arm stops short of target | Raise `kP` |
| Arm oscillates at target | Lower `kP`, raise `kD` |
| `Intake/Pivot At Target` never goes true | Arm not reaching position. Check kP, kG, and position values. |
| Roller not pulling balls in | Raise `INTAKE_POWER` |
| Roller sounds strained or ball jams | Lower `INTAKE_POWER`, check mechanical compression |
| Arm grinding into bumper hard stop | Lower `SOFT_LIMIT_FORWARD` |
| Roller starts before arm arrives in sequence | Pivot too slow or `PIVOT_TIMEOUT_SECONDS` too short |
| Brownouts during arm movement | Lower `PIVOT_SUPPLY_CURRENT_LIMIT` |
| Arm feels weak, can't hold position | Raise `PIVOT_STATOR_CURRENT_LIMIT` |
| Position off after power-on | Arm was not at stow on boot. Power off, reposition, reboot. |

---

Team 8041 - Lake City Ultrabots, 2026
