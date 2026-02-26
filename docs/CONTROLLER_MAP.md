# Controller Map - Team 8041

Driver controller is port 0. Xbox controller. All bindings are in `RobotContainer.configureBindings()`.
---

## Full Binding Table

| Input      | Action           | Trigger Type |
|-------|--------|--------------|
| Left Stick | Drive (forward/back/strafe) | Always active |
| Right Stick X | Rotate | Always active (when auto-aim is off) |
| Left Bumper | Toggle auto-aim on/off | Press once to toggle |
| Right Bumper | Reset field-centric heading | Press once |
| Left Trigger | Shooter eject | Hold |
| Right Trigger | Shoot | Hold, idles on release |
| START | Align to hub | Hold |
| Y | Align to tower | Hold |
| X | Align to outpost | Hold |
| B | Passing shot | Hold |
| A | Pivot to travel position | Press once |
| D-Up | Pivot to stow | Press once |
| D-Down | Pivot to intake position | Press once |
| D-Left | Start intake roller | Press once |
| D-Right | Stop intake roller | Press once |

### Currently unbound (available)
- BACK button
- Left Stick button (press down)
- Right Stick button (press down)
- Right Stick Y axis

---

## What Each Input Does and Why

---

### Left Stick - Drive

The left stick controls all translation. Pushing forward drives forward, pushing right drives right, any combination works. This is field-centric, meaning forward on the stick is always away from the driver no matter which direction the robot is facing.

The robot uses asymmetric slew rate limiting on this input. It accelerates quickly but decelerates at a controlled rate to prevent tipping. See `docs/DRIVE_TUNING.md` for tuning those rates.

When auto-aim is on and a hub tag is visible, rotation is taken over by the vision PID. The driver still has full control of left stick translation.

---

### Right Stick X - Rotate

Only the X axis (left/right) of the right stick is used. This controls the robot's rotation rate. The Y axis is unused.

When auto-aim is active, this input is ignored for rotation. The vision PID controls heading instead.

---

### Left Bumper - Toggle Auto-Aim

Toggles auto-aim on or off each press. When on, the robot locks its heading onto the nearest visible hub AprilTag using a PID loop. The driver keeps full drive control with the left stick.

Auto-aim also sets the shooter to the correct speed for the current distance to the hub automatically, so the driver does not have to think about shooter RPM while moving.

When auto-aim is on, `AutoAim/Enabled` on SmartDashboard shows true. When no hub tag is visible, auto-aim does nothing even if toggled on.

---

### Right Bumper - Reset Field-Centric Heading

Resets the field-centric reference so that wherever the robot is currently pointing becomes the new "forward" for the left stick.

Use this if the robot gets bumped hard and the heading drifts, or if the driver loses track of which direction the robot considers forward. Press it while pointing toward the far end of the field to restore normal orientation.

---

### Left Trigger - Eject

Runs the shooter eject sequence while held. This reverses the shooter to push fuel back out. Used if a piece gets stuck or jammed in the shooter.

Runs the full `ejectSequence` command which handles the shooter state machine.

---

### Right Trigger - Shoot

Spins the shooter up to full speed while held. The shooter is ready to fire when `Shooter/At Target` on SmartDashboard shows true. Release to idle the shooter.

The sequence is: hold trigger to spin up, wait for at-target, fuel feeds through. Release and the shooter returns to idle.

If auto-aim is on at the same time, rotation is still being controlled by vision. This is the normal scoring workflow: toggle auto-aim on, drive toward hub, hold right trigger when aligned.

---

### START - Align to Hub

Runs full three-axis alignment to the nearest hub AprilTag while held. Controls rotation, forward/back distance, and strafe simultaneously. The driver can still push the left stick to override the strafe axis if they need to arc around defense.

Alliance-aware. Automatically aligns to red hub tags if red alliance, blue hub tags if blue alliance. Resolved at match start from DriverStation, not hardcoded.

Alignment completes when rotation, distance, and lateral offset are all within tolerance for 0.5 seconds. Times out after 8 seconds if it can't get there.

---

### Y - Align to Tower

Same three-axis alignment as START but targets the tower AprilTags for climbing approach. Aligns to the correct alliance side automatically.

Tower target distance is set to 36 inches. Tune this in `VisionConstants.APRILTAG_DISTANCES` once you test the climbing approach distance.

---

### X - Align to Outpost

Same three-axis alignment targeting the outpost AprilTags. This is the human player station where fuel is handed off.

Outpost target distance is 30 inches. Tune this in `VisionConstants.APRILTAG_DISTANCES` once you confirm the right handoff distance with your human player.

---

### B - Passing Shot

Spins the shooter to passing speed (`PASS_VELOCITY` / `FLYWHEEL_PASS_VELOCITY`) while held. Lower speed and lower flywheel ratio than a scoring shot, which gives a flatter faster trajectory for long distance passes across the field.

The intent is the driver holds B from center field, waits for `Shooter/At Target`, and fires. Release to idle.

See `docs/SHOOTER_TUNING.md` for pass speed tuning.

---

### A - Pivot to Travel

Commands the intake arm to the travel position and stops there. This is a manual safety press. Travel sits partway between stow and intake, high enough to clear the 15 degree ramp if needed.

Whether TRAVEL is actually needed depends on testing on the ramp. If the arm at intake position doesn't contact the ramp, this button becomes less important but is still available.

Does not affect the intake roller. Roller keeps whatever state it was in.

---

### D-Up - Pivot to Stow

Commands the intake arm back inside the robot to the stow position. Use this after finishing intake or before climbing.

Does not stop the intake roller. If you're done intaking, press D-Right first to stop the roller, then D-Up to stow.

---

### D-Down - Pivot to Intake

Commands the intake arm out to the intake position. The arm will swing down to where it can pick up fuel from the carpet.

Does not start the intake roller. Press D-Left separately to start spinning. Or use the full sequence in autonomous (`Start Intake`) which does both in order and waits for the arm to arrive before spinning the roller.

---

### D-Left - Start Intake Roller

Starts the intake roller at `INTAKE_VELOCITY`. Does not move the pivot. If the arm is still at stow when you press this, the roller just spins inside the robot and does nothing.

Normal workflow for intake: D-Down to deploy the arm, D-Left to start the roller once the arm is out.

---

### D-Right - Stop Intake Roller

Sets the intake roller to idle (stopped). Does not move the pivot.

---

## Trigger Behavior Types Explained

There are two main trigger types used in the bindings. This matters because they behave differently:

**whileTrue** - the command runs as long as you hold the button. The moment you release, the command is cancelled and any `onFalse` action fires. Right trigger (shoot) uses this so releasing the trigger immediately idles the shooter.

**onTrue** - the command runs once when you press the button and then ends on its own. Used for pivot positions and intake roller because you just want to set the state and leave it there. You press D-Down once and the arm goes to intake and stays there even after you let go.

---

## Auto-Aim Workflow (Normal Scoring)

1. Toggle auto-aim on with Left Bumper. `AutoAim/Enabled` shows true on SmartDashboard.
2. Drive toward the hub with the left stick. The robot rotates to face the hub automatically.
3. Shooter speed adjusts automatically based on distance.
4. Hold Right Trigger to shoot when `Shooter/At Target` shows true.
5. Release Right Trigger. Shooter idles.
6. Toggle auto-aim off with Left Bumper when done scoring.

---

## Manual Scoring Workflow (No Auto-Aim)

1. Hold START to run full vision alignment to hub. Robot handles rotation, distance, and centering.
2. Hold Right Trigger to shoot when aligned and `Shooter/At Target` shows true.
3. Release both.

---

## Intake Workflow (Teleop)

1. Press D-Down to deploy the arm to intake position.
2. Drive to the fuel.
3. Press D-Left to start the roller.
4. Press D-Right to stop the roller when done.
5. Press D-Up to stow the arm.

---

## Notes for Driver Practice

- Auto-aim and vision alignment (START/Y/X) can run at the same time but auto-aim only controls rotation. If you hold START while auto-aim is on, alignment still works because alignment takes full control of the drivetrain.
- If the robot loses field-centric orientation after a collision, press Right Bumper to reset.
- The passing shot (B) is completely independent from auto-aim. You handle aim manually for passes.
- Shooter eject (Left Trigger) will override a normal shoot if both triggers are held. Avoid doing that.

---

Team 8041 - Lake City Ultrabots, 2026
