# Controller Map - Team 8041

Driver controller is port 0. Xbox controller. All bindings are in `RobotContainer.configureBindings()`.

---

## Full Binding Table

| Input | Action | Trigger Type |
|-------|--------|--------------|
| Left Stick | Drive (forward/back/strafe) | Always active |
| Right Stick X | Rotate | Always active (when auto-aim is off) |
| Left Bumper | Toggle auto-aim on/off | Press once to toggle |
| Right Bumper | Reset field-centric heading | Press once |
| Left Trigger | Shooter eject | Hold, idles on release |
| Right Trigger | Shoot | Hold, idles on release |
| START | Align to hub | Hold |
| BACK | Align to tower | Hold |
| Y | Align to outpost | Hold |
| X | Intake roller | Hold, idles on release |
| B | Passing shot | Hold, idles on release |
| A | Pivot to travel position | Press once |
| D-Up | Pivot to stow | Press once |
| D-Down | Pivot to intake position | Press once |
| D-Left | Follow tag demo | Hold |

### Currently unbound (available)
- D-Right
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

Auto-aim uses the shooter camera (`limelight-april`) only.

---

### Right Bumper - Reset Field-Centric Heading

Resets the field-centric reference so that wherever the robot is currently pointing becomes the new "forward" for the left stick.

Use this if the robot gets bumped hard and the heading drifts, or if the driver loses track of which direction the robot considers forward. Press it while pointing toward the far end of the field to restore normal orientation.

**Important:** Because the Pigeon IMU's forward is the climber/intake side of the robot (not the shooter side), the driver will likely need to press this at the start of each match to set their preferred forward direction.

---

### Left Trigger - Eject

Runs the shooter eject while held. This reverses the feed to push fuel back out. Used if a piece gets stuck or jammed in the shooter. Idles the shooter on release.

---

### Right Trigger - Shoot

Spins the shooter flywheel and runs the feed rollers while held. Release to idle the shooter.

If auto-aim is on at the same time, rotation is still being controlled by vision. This is the normal scoring workflow: toggle auto-aim on, drive toward hub, hold right trigger when aligned.

---

### START - Align to Hub

Runs full three-axis alignment to the nearest hub AprilTag while held. Controls rotation, forward/back distance, and strafe simultaneously. The driver can still push the left stick to override the strafe axis if they need to arc around defense.

Alliance-aware. Automatically aligns to red hub tags if red alliance, blue hub tags if blue alliance. Resolved at match start from DriverStation, not hardcoded.

Alignment completes when rotation, distance, and lateral offset are all within tolerance for 0.5 seconds. Times out after 8 seconds if it cannot get there.

Uses the shooter camera (`limelight-april`).

---

### BACK - Align to Tower

Same three-axis alignment as START but targets the tower AprilTags for climbing approach. Aligns to the correct alliance side automatically.

Tower target distance is set to 18 inches. Tune this in `VisionConstants.APRILTAG_DISTANCES` once you test the climbing approach distance.

Uses the shooter camera (`limelight-april`).

---

### Y - Align to Outpost

Same three-axis alignment targeting the outpost AprilTags. This is the human player station where fuel is handed off.

Outpost target distance is 14 inches. Tune this in `VisionConstants.APRILTAG_DISTANCES` once you confirm the right handoff distance with your human player.

Uses the shooter camera (`limelight-april`).

---

### X - Intake Roller

Runs the intake roller while held. When you release X, the roller stops automatically (idles on release).

This does not move the pivot arm. If the arm is still at stow when you press this, the roller just spins inside the robot and does nothing. Deploy the arm first with D-Down.

---

### B - Passing Shot

Spins the shooter to passing speed (`FLYWHEEL_PASS_RPS`) while held. Lower speed than a scoring shot, which gives a flatter faster trajectory for long distance passes across the field. Idles on release.

See `docs/SHOOTER_TUNING.md` for pass speed tuning.

---

### A - Pivot to Travel

Commands the intake arm to the travel position and stops there. Travel sits partway between stow and intake, high enough to clear the 15 degree ramp if needed.

Does not affect the intake roller. Roller keeps whatever state it was in.

---

### D-Up - Pivot to Stow

Commands the intake arm back inside the robot to the stow position. Use this after finishing intake or before climbing.

Does not stop the intake roller.

---

### D-Down - Pivot to Intake

Commands the intake arm out to the intake position. The arm will swing down to where it can pick up fuel from the carpet.

Does not start the intake roller. Press X to start spinning.

---

### D-Left - Follow Tag Demo

Hold to make the robot follow any visible AprilTag at a safe distance. This is for outreach events and demos only, not competition. Uses conservative speed limits so the robot does not crash into anything.

Uses the shooter camera (`limelight-april`). Stops immediately when the button is released.

---

## Trigger Behavior Types Explained

There are two main trigger types used in the bindings. This matters because they behave differently:

**whileTrue** - the command runs as long as you hold the button. The moment you release, the command is cancelled and any `onFalse` action fires. Right trigger (shoot), left trigger (eject), B (pass), and X (intake) all use this pattern so releasing the button immediately idles the mechanism.

**onTrue** - the command runs once when you press the button and then the state stays. Used for pivot positions (D-Up, D-Down, A) because you just want to set the position and leave it there. You press D-Down once and the arm goes to intake and stays even after you let go.

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
3. Hold X to run the intake roller.
4. Release X when done (roller stops automatically).
5. Press D-Up to stow the arm.

---

## Notes for Driver Practice

- Auto-aim and vision alignment (START/BACK/Y) can run at the same time but auto-aim only controls rotation. If you hold START while auto-aim is on, alignment still works because alignment takes full control of the drivetrain.
- If the robot loses field-centric orientation after a collision, press Right Bumper to reset.
- The passing shot (B) is completely independent from auto-aim. You handle aim manually for passes.
- Shooter eject (Left Trigger) will override a normal shoot if both triggers are held. Avoid doing that.
- Remember that the Pigeon "forward" is the climber side. Hit Right Bumper to recenter when needed.

---

Team 8041 - Lake City Ultrabots, 2026
