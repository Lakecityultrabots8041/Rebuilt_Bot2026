# Intake Tuning Guide

All tunable values live in: `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`

---

## How the Mechanism Works

The intake has two motors running independently:

**Pivot motor** (ID 2, CANivore "Jeffery") - this is the arm that swings out and back. It uses Motion Magic, which is a position controller that follows a smooth motion profile. You set where you want it to go and Motion Magic figures out how to get there at a controlled speed.

**Intake motor** (ID 3, CANivore "Jeffery") - this is the roller that pulls fuel in. It runs at a set velocity and just spins.

The pivot has three positions:
- **STOW** - arm inside the robot, legal starting position
- **INTAKE** - arm extended to the ground where fuel is
- **TRAVEL** - arm held partway up, used as a manual safety press if needed

The arm physically rests against the robot bumper if it tries to go past intake position. The soft limit in code stops the motor from grinding against that hard stop continuously.

On power-on, the code seeds the encoder to STOW_POSITION and immediately commands the arm to hold stow. The arm must be physically placed at stow before powering on.

---

## First Time Setup - Verify Before Tuning Anything

Before touching any gains, confirm the basics:

1. Power on with arm physically at stow (inside the robot)
2. Open SmartDashboard and confirm `Intake/Pivot Position` reads close to 0.0
3. Press DPad Down (pivot to intake) and watch `Intake/Pivot Position` increase
4. Press DPad Up (pivot to stow) and watch it return to 0.0
5. Confirm the arm moves smoothly and stops at both positions without banging hard

If the arm moves the wrong direction when commanded, the motor is inverted. Set `pivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive` in `IntakeSubsystems.java`.

If the arm drifts after stopping, brake mode may not have applied. Check that the motor ID and CAN bus name match the physical hardware.

---

## Step 1 - Find the Real Position Values

The default positions in IntakeConstants are placeholders and will need to be measured:

| Constant | Default | What it is |
|---|---|---|
| `STOW_POSITION` | 0.0 | Where the encoder reads when arm is at stow. Always 0.0 because we seed the encoder there on boot. |
| `INTAKE_POSITION` | 5.0 | Where the encoder reads when arm is fully extended to intake fuel. Measure this. |
| `TRAVEL_POSITION` | 3.5 | Where the encoder reads at the safe intermediate position. Measure this. |

### How to measure INTAKE_POSITION

1. Power on with arm at stow
2. Manually command the arm to intake position using DPad Down
3. Watch `Intake/Pivot Position` on SmartDashboard as the arm moves
4. When the arm is at the right height to pick up fuel from the carpet, read that number
5. Put that number into `INTAKE_POSITION` in IntakeConstants

### How to measure TRAVEL_POSITION

Same process. Command the arm to a height that clears the 15 degree ramp safely and read the number. This is a judgment call on the field, especially after you test the ramp.

### After measuring, update SOFT_LIMIT_FORWARD

Once you have the real `INTAKE_POSITION`, set `SOFT_LIMIT_FORWARD` to about 0.5 above it. This gives the arm a small buffer past intake before the soft limit kicks in, so Motion Magic can settle at the target without fighting the limit, but the motor won't push hard against the bumper indefinitely.

---

## Step 2 - Tune Motion Magic (Arm Speed)

Motion Magic controls how fast the arm moves and how smoothly it gets there. Three values in IntakeConstants control the motion profile:

| Constant | Default | What it controls |
|---|---|---|
| `CRUISE_VELOCITY` | 15 | Max speed the arm moves at during travel (rot/sec) |
| `ACCELERATION` | 20 | How fast it ramps up to cruise speed (rot/sec2) |
| `JERK` | 20 | How smoothly acceleration changes (rot/sec3) |

You want the arm to come out fast. Raise `CRUISE_VELOCITY` first to get speed, then raise `ACCELERATION` to get it up to that speed faster. Jerk smooths out the start and stop - lower jerk = smoother but slower feeling start.

### Tuning process

1. Start with defaults: CRUISE = 15, ACCEL = 20, JERK = 20
2. Command pivot to intake, time how long it takes
3. Raise `CRUISE_VELOCITY` to 25 and repeat
4. Keep raising until the arm moves as fast as you want without overshooting the target position
5. If the arm slams into position and bounces, the PID needs tuning (section below), not Motion Magic

Watch `Intake/Pivot At Target` on SmartDashboard to confirm it goes true after each move.

---

## Step 3 - Tune the Pivot PID

The pivot uses these gains. They work together to get the arm to its target accurately:

| Gain | Default | Role |
|---|---|---|
| `kG` | 4.25 | Gravity compensation. Holds the arm against gravity at any angle. |
| `kS` | 2.0 | Static friction. Overcomes the initial resistance before the arm starts moving. |
| `kV` | 4.5 | Velocity feedforward. Main effort while the arm is in motion. |
| `kA` | 0.01 | Acceleration feedforward. Helps during fast changes in speed. |
| `kP` | 30 | Proportional. Corrects remaining position error at the target. |
| `kI` | 0 | Integral. Leave at 0 unless the arm consistently stops short of target. |
| `kD` | 0.1 | Derivative. Damps oscillation at the target. |

### Tuning order

**kG first** - this is the most important one for a gravity arm. Set everything else to 0 first. Increase kG until the arm holds its position without drifting when you hold it in the air and release. If it drifts down, raise kG. If it creeps up, lower kG. The Arm_Cosine gravity type means the code automatically adjusts kG compensation based on the arm angle, so you just need to find the right value and it handles the rest.

**kS second** - with kG set, command a move. If the arm hesitates before starting to move, raise kS slightly. Typical: 1.0-3.0.

**kV third** - if the arm moves too slow during the profile, raise kV. If it runs past the target consistently, lower it. Typical: 3.0-6.0.

**kP last** - with the feedforwards set, the arm should be close to target on its own. kP handles the final correction. Raise from 0 until the arm snaps to the target cleanly. Too high and it oscillates. Too low and it stops a bit short.

### SmartDashboard values to watch

| Key | What to look for |
|---|---|
| `Intake/Pivot Position` | Should match the target position when at rest |
| `Intake/Pivot At Target` | Should go true after each move and stay true |
| `Intake/Pivot State` | Confirms the state machine is in the right state |

---

## Step 4 - Tune the Intake Roller

The roller is simpler. It just needs enough speed to pull fuel in reliably without being so fast that it throws fuel out the other side.

| Constant | Default | What it is |
|---|---|---|
| `INTAKE_VELOCITY` | 50.0 RPS | Speed to run when intaking |
| `EJECT_VELOCITY` | -30.9 RPS | Speed to run when ejecting (negative = reverse) |
| `IDLE_VELOCITY` | 0.0 | Stopped |

### Tuning process

1. Set arm to intake position
2. Place fuel in front of the intake
3. Run intake (DPad Left) and watch whether fuel gets pulled in smoothly
4. If fuel slips off without being pulled in, raise `INTAKE_VELOCITY`
5. If fuel bounces around violently, lower it

The intake motor PID gains (kP = 0.1, kV = 0.1, kS = 0.1) in `IntakeSubsystems.java` are very rough placeholders. If the roller velocity isn't hitting the target:
- Raise kV if it runs consistently under target speed
- Raise kP if it oscillates or has steady-state error
- These will likely need proper tuning once on the real robot

---

## Step 5 - Confirm Sequences Work End to End

The intake sequences (Start Intake, End Intake) are what autonomous uses. Test these in teleop first:

### Starting Intake Sequence (PathPlanner: "Start Intake")
1. Commands pivot to intake position
2. Waits until pivot actually reaches the target (up to 2 seconds)
3. Starts the intake roller

If the roller starts before the arm arrives, `PIVOT_TIMEOUT_SECONDS` timed out too fast or the pivot is too slow. Check `Intake/Pivot At Target` on SmartDashboard.

### Ending Intake Sequence (PathPlanner: "End Intake")
1. Stops the roller immediately
2. Commands pivot to travel position

---

## Current Limits

The pivot has current limits to protect the motor and mechanism if something goes wrong.

| Constant | Default | What it does |
|---|---|---|
| `PIVOT_STATOR_CURRENT_LIMIT` | 40 A | Max torque the pivot motor produces. Prevents grinding against the bumper hard stop. |
| `PIVOT_SUPPLY_CURRENT_LIMIT` | 30 A | Max current drawn from the battery per motor. |

If the arm feels weak and can't hold position against gravity, raise stator limit in small steps. If you're seeing brownouts, lower supply limit.

---

## Soft Limits

Soft limits stop the motor from commanding past the physical range of the arm.

| Constant | Default | What it does |
|---|---|---|
| `SOFT_LIMIT_FORWARD` | 5.5 | Motor won't command past this position (toward intake). Set 0.3-0.5 above INTAKE_POSITION. |
| `SOFT_LIMIT_REVERSE` | 0.0 | Motor won't command below this position (toward stow). Matches STOW_POSITION. |

After you measure the real INTAKE_POSITION, update SOFT_LIMIT_FORWARD to be slightly above it. If the arm is hitting the bumper hard stop and you can hear the motor straining, the soft limit is too high.

---

## Pre-Match Checklist

- [ ] Arm physically placed at stow before powering on
- [ ] `Intake/Pivot Position` reads 0.0 (or close) on boot
- [ ] Pivot moves to intake and back to stow on command
- [ ] `Intake/Pivot At Target` goes true at each position
- [ ] Intake roller pulls fuel in cleanly at intake position
- [ ] Start Intake sequence runs all the way through without roller starting early
- [ ] End Intake sequence returns arm to travel position
- [ ] No brownouts during intake motor operation

---

## Quick Reference - What to Change for Each Problem

| Problem | Change |
|---|---|
| Arm moves wrong direction | Add or remove inversion in IntakeSubsystems pivot config |
| Arm drifts down when holding position | Raise kG |
| Arm creeps up when holding position | Lower kG |
| Arm hesitates before moving | Raise kS |
| Arm too slow deploying | Raise CRUISE_VELOCITY |
| Arm overshoots and bounces | Lower kP or lower CRUISE_VELOCITY |
| Arm stops short of target | Raise kP |
| Arm oscillates at target | Lower kP, raise kD |
| Roller not pulling fuel in | Raise INTAKE_VELOCITY |
| Brownouts during intake | Lower PIVOT_SUPPLY_CURRENT_LIMIT |
| Arm grinding on bumper hard stop | Lower SOFT_LIMIT_FORWARD |
| Roller starts before arm arrives | Check PIVOT_TIMEOUT_SECONDS, or pivot PID is too slow |

---

Team 8041 - Lake City Ultrabots, 2026
