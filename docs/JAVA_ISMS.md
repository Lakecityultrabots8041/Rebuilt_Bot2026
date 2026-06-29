# Java-isms - Patterns Used in Our Code

Chris Shaw - Lake City Ultrabots Programming Mentor
2026 REBUILT season

This doc explains the Java and WPILib patterns that show up in our vision code. If you read [VISION.md](VISION.md) and hit a line of code that looks like magic, find it here.

Each entry tells you three things: what the pattern is, why we used it, and where to click to see it in the real code. In VS Code you can Ctrl+Click (Cmd+Click on Mac) any of the links below to jump straight to the file and line.

---

## Quick Index

- [`final` fields and classes](#final-fields-and-classes)
- [Private constructor (utility class)](#private-constructor-utility-class)
- [Static initializer block](#static-initializer-block)
- [`Set.of(...)` immutable sets](#setof-immutable-sets)
- [`Optional<T>`](#optionalt)
- [Ternary operator](#ternary-operator)
- [`Supplier<T>` and method references](#suppliert-and-method-references)
- [`@FunctionalInterface`](#functionalinterface)
- [`record`](#record)
- [Lambdas](#lambdas)
- [`var` (local type inference)](#var-local-type-inference)
- [`@Override` and the Command lifecycle](#override-and-the-command-lifecycle)
- [`SubsystemBase` and `periodic()`](#subsystembase-and-periodic)
- [Per-loop caching and preallocation](#per-loop-caching-and-preallocation)
- [WPILib helpers we lean on](#wpilib-helpers-we-lean-on)

---

## `final` fields and classes

**What it is:** `final` means "this cannot be reassigned after it is set." On a field, the value is locked once the constructor finishes. On a class, nobody can extend it.

**Why we use it:** It documents intent and catches mistakes. If a field should never change after construction (like which camera a command talks to), `final` makes the compiler enforce that. [`VisionConstants`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java) is `public final class` because it is a bag of constants, not something you should subclass.

**Where to see it:** [`Limelight_Move.java:26-29`](../src/main/java/frc/robot/commands/Limelight_Move.java#L26) - `drivetrain`, `limelight`, and the suppliers are all `final`.

Back to vision: [Three-Axis Alignment](VISION.md#three-axis-alignment-limelight_move).

---

## Private constructor (utility class)

**What it is:** A `private` constructor stops anyone from writing `new VisionConstants()`.

**Why we use it:** [`VisionConstants`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java) only holds static values and static methods. There is no reason to ever make an instance of it, so we block it. This is a standard Java pattern for "constants holder" or "static helper" classes.

**Where to see it:** [`VisionConstants.java:20`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L20) - `private VisionConstants() {}`.

---

## Static initializer block

**What it is:** A `static { ... }` block runs once, when the class is first loaded, before any code uses it.

**Why we use it:** Our target distances live in one array (`APRILTAG_DISTANCES`) indexed by tag ID. We want every entry filled in with sensible numbers before anyone reads it. A loop fills the whole array with a default, then we overwrite the specific hub/tower/outpost/trench tags. That setup logic does not fit in a single `= ...` assignment, so it goes in a static block.

**Where to see it:** [`VisionConstants.java:25-78`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L25).

Back to vision: [Target Distances](VISION.md#target-distances).

---

## `Set.of(...)` immutable sets

**What it is:** `Set.of(2, 3, 4)` builds a small, read-only set. You cannot add to it or remove from it later.

**Why we use it:** Our tag groups (which AprilTags belong to the red hub, the blue tower, etc.) never change at runtime. Making them immutable means no code can accidentally corrupt them, and a `Set` gives us a fast `.contains(tagID)` check when we ask "is the tag I see one of mine?"

**Where to see it:** [`VisionConstants.java:85-104`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L85), and the lookup at [`Limelight_Move.java:90`](../src/main/java/frc/robot/commands/Limelight_Move.java#L90) (`acceptedTagIDs.contains(tagID)`).

Back to vision: [Alliance-Aware Tag Selection](VISION.md#alliance-aware-tag-selection).

---

## `Optional<T>`

**What it is:** `Optional<Alliance>` is a box that either holds a value or is empty. It forces you to handle the "no value yet" case instead of crashing on a null.

**Why we use it:** Before a match, the DriverStation may not know our alliance color yet. `DriverStation.getAlliance()` returns an `Optional`, so we check `isPresent()` before reading it. No alliance set means we fall back to blue.

**Where to see it:** [`VisionConstants.java:107-110`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L107).

Back to vision: [Alliance-Aware Tag Selection](VISION.md#alliance-aware-tag-selection).

---

## Ternary operator

**What it is:** `condition ? a : b` is a one-line if/else that returns a value. "If condition, use a, otherwise use b."

**Why we use it:** Short, readable choices. Picking the red or blue tag set is a perfect fit: `return isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;`. The direction multiplier for front vs rear camera is another: `limelight.isRearFacing() ? -1.0 : 1.0`.

**Where to see it:** [`VisionConstants.java:113`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L113) and [`Limelight_Move.java:61`](../src/main/java/frc/robot/commands/Limelight_Move.java#L61).

---

## `Supplier<T>` and method references

**What it is:** A `Supplier<T>` is a tiny object that knows how to *produce* a value when you call `.get()` on it - but not until then. A method reference like `VisionConstants::getHubTags` hands the method itself to something, instead of calling it now.

**Why we use it (this is the important one):** The field is mirrored, so we must align to *our* alliance's tags. But during autonomous we might not know our color until the match starts. If we passed the actual tag set into the alignment command when the robot boots, we would lock in whatever color happened to be set at boot - usually wrong.

Instead we pass a `Supplier` (`VisionConstants::getHubTags`). The command holds the recipe, not the result. Every time the command starts, it calls `.get()` and asks fresh: "what alliance are we *right now*?" That is why the alignment always targets the correct tags even if the color is set late.

```java
// RIGHT - asks for the alliance fresh every time the command starts
new Limelight_Move(drivetrain, limelightShooter, VisionConstants::getHubTags, ...);

// WRONG - evaluates once at boot, locks to whatever alliance was set then
new Limelight_Move(drivetrain, limelightShooter, () -> VisionConstants.BLUE_HUB_TAGS, ...);
```

**Where to see it:** the factories in [`RobotContainer.java:77-107`](../src/main/java/frc/robot/RobotContainer.java#L77), and where the command actually calls `.get()` at [`Limelight_Move.java:74`](../src/main/java/frc/robot/commands/Limelight_Move.java#L74).

We also use suppliers for live robot state: `() -> drivetrain.getState().Pose` hands the camera a way to ask "where are we?" every loop without the camera needing a reference to the drivetrain. See [`RobotContainer.java:112-122`](../src/main/java/frc/robot/RobotContainer.java#L112).

Back to vision: [Alliance-Aware Tag Selection](VISION.md#alliance-aware-tag-selection).

---

## `@FunctionalInterface`

**What it is:** An interface with exactly one method. The `@FunctionalInterface` annotation tells the compiler to enforce that, and lets you create one with a lambda instead of a whole class.

**Why we use it:** We needed a clean way to hand vision poses back to the drivetrain without the camera knowing what a drivetrain is. `VisionPoseConsumer` is our own one-method interface: "give me a pose, a timestamp, and an accuracy." RobotContainer fills it in with a lambda that forwards to `drivetrain.addVisionMeasurement(...)`. This keeps the camera code decoupled from the drivetrain.

`DoubleSupplier` (built into Java) is the same idea, specialized to return a `double` - we use it for the driver's strafe stick value.

**Where to see it:** the interface at [`VisionConstants.java:188-191`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L188), and it being filled in at [`RobotContainer.java:113`](../src/main/java/frc/robot/RobotContainer.java#L113).

Back to vision: [How the Subsystem Supports Two Cameras](VISION.md#how-the-subsystem-supports-two-cameras).

---

## `record`

**What it is:** A `record` is a short way to declare a small, immutable data holder. One line gives you the fields, a constructor, and getters.

**Why we use it:** In simulation we precompute every tag's `(id, x, y)` once and store them in a list. We do not need a full class for that - a `record SimTag(int id, double x, double y) {}` is enough, and it makes clear the data never changes.

**Where to see it:** [`LimelightSubsystem.java:70`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L70).

Back to vision: [Simulation](VISION.md#simulation).

---

## Lambdas

**What it is:** A lambda is an inline function: `(x) -> x * 2`. It is the short form for "a small piece of behavior I want to pass around."

**Why we use it:** Most of our suppliers and consumers are written as lambdas. `() -> -controller.getLeftX()` is a zero-argument function that reads the driver stick and negates it. `(pose, timestamp, accuracy) -> drivetrain.addVisionMeasurement(...)` is a three-argument function that forwards a measurement.

**Where to see it:** all through [`RobotContainer.java:112-122`](../src/main/java/frc/robot/RobotContainer.java#L112) and the default drive command at [`RobotContainer.java:168-206`](../src/main/java/frc/robot/RobotContainer.java#L168).

---

## `var` (local type inference)

**What it is:** `var` lets the compiler figure out a local variable's type from the right-hand side. `var result = LimelightHelpers.getBotPoseEstimate(...)` is the same as writing the full type name.

**Why we use it:** Some WPILib and CTRE types have long names. `var` keeps lines short when the type is obvious from context. It only works on local variables, never on fields or method parameters.

**Where to see it:** [`LimelightSubsystem.java:220`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L220) and [`:337`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L337).

---

## `@Override` and the Command lifecycle

**What it is:** `@Override` tells the compiler "I am replacing a method from the parent class." If you spell the name wrong, it errors instead of silently doing nothing.

**Why it matters here:** A WPILib `Command` has four lifecycle methods the scheduler calls for you:

- `initialize()` - runs once when the command starts
- `execute()` - runs every loop (about every 20 ms) while active
- `end(boolean interrupted)` - runs once when the command stops
- `isFinished()` - checked every loop; returning true ends the command

Knowing which code goes in which method is most of understanding a command. In our alignment command: we read the alliance tags in `initialize()`, drive the motors in `execute()`, stop the motors and turn the LED off in `end()`, and check the "aligned long enough or timed out" condition in `isFinished()`.

**Where to see it:** [`Limelight_Move.java:72-195`](../src/main/java/frc/robot/commands/Limelight_Move.java#L72).

Back to vision: [When is it "aligned"?](VISION.md#when-is-it-aligned)

---

## `SubsystemBase` and `periodic()`

**What it is:** Anything that extends `SubsystemBase` gets a `periodic()` method that WPILib runs every loop automatically. It is the heartbeat of a subsystem.

**Why we use it:** Our camera reads NetworkTables, updates its caches, runs the vision-fusion filters, and pushes dashboard values - all once per loop, all inside `periodic()`. You never call it yourself; the scheduler does.

**Where to see it:** [`LimelightSubsystem.java:237-269`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L237).

Back to vision: [How Distance Works](VISION.md#how-distance-works).

---

## Per-loop caching and preallocation

**What it is:** Reading the same value once per loop and storing it in a field, instead of reading it many times. And building objects once at startup instead of every loop.

**Why we use it:** The robot loop runs about 50 times a second. Anything you do inside it happens a lot. Two habits keep that cheap:

1. **Read NetworkTables once.** At the top of `periodic()` we read each value (`tx`, `ty`, `ta`, ...) one time into a `cached...` field, then everything else uses the cached copy. Reading NetworkTables repeatedly in one loop would be wasteful and could give inconsistent values mid-loop.
2. **Do not allocate in the loop.** Creating new objects every loop makes the garbage collector run more often, which can cause stutters. So the simulation tag list is built once in the constructor, and the alignment command reuses one `ChassisSpeeds` object instead of making a new one each loop.

**Where to see it:** the per-loop NT cache at [`LimelightSubsystem.java:242-247`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L242), the prebuilt sim cache at [`:100-108`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L100), and the reused speeds object at [`Limelight_Move.java:37`](../src/main/java/frc/robot/commands/Limelight_Move.java#L37).

---

## WPILib helpers we lean on

These are not Java-the-language, but they show up constantly in robot code. Worth knowing.

| Helper | What it does | Where we use it |
|--------|--------------|-----------------|
| `MathUtil.clamp(x, min, max)` | Forces a number to stay inside a range. We clamp every motor output so a big error cannot command full speed. | [`Limelight_Move.java:118`](../src/main/java/frc/robot/commands/Limelight_Move.java#L118) |
| `Units.inchesToMeters(...)` / `metersToInches(...)` | Converts units. We keep all internal math in meters and only convert to inches for the dashboard. Mixing units is a classic bug source. | [`VisionConstants.java:31`](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java#L31) |
| `PIDController` with `enableContinuousInput(-PI, PI)` | A PID loop that knows angles wrap around. Without it, turning from 359 to 1 degree would take the long way around. | [`RobotContainer.java:123`](../src/main/java/frc/robot/RobotContainer.java#L123) |
| `SlewRateLimiter` | Caps how fast a value can change. We use it so auto-aim accelerates smoothly instead of jerking. | [`RobotContainer.java:71`](../src/main/java/frc/robot/RobotContainer.java#L71) |
| `VecBuilder.fill(a, b, c)` | Packs numbers into the small matrix CTRE wants for vision uncertainty: `[x, y, rotation]`. | [`LimelightSubsystem.java:337`](../src/main/java/frc/robot/subsystems/vision/LimelightSubsystem.java#L337) |

Back to vision: [Auto-Aim](VISION.md#auto-aim) and [Vision-Odometry Fusion](VISION.md#vision-odometry-fusion).

---

Team 8041 - Lake City Ultrabots, 2026
