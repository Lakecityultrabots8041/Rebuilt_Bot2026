# LED Subsystem Guide

## How the CANdle Works

The CTRE CANdle controls LEDs over CAN bus using Phoenix 6.
It has 8 onboard RGB LEDs and drives an external WS2812B strip.
Patterns are set with `candle.setControl()` using animation or solid color objects.
All animation runs on the CANdle hardware, not the RoboRIO.

Hardware is on the default CAN bus, CAN ID 1 (configured in LEDConstants.java).

## Wiring

- CANdle CAN H/L to RoboRIO CAN bus
- CANdle LED data out to strip DATA IN
- CANdle 5V rail or VRM 5V/2A to strip +5V
- Common ground between CANdle, strip, and power source

## LED Indices

| Range  | What                                         |
|--------|----------------------------------------------|
| 0-7    | Onboard CANdle LEDs (always RGB)             |
| 8-32   | External strip (25 LEDs, configured as GRB)  |

All patterns use the full range (0-32) so onboard LEDs match the strip.

## Priority Waterfall

The LED picks one pattern per loop cycle using a first-match-wins priority system.
The highest-priority condition that is true wins. Active robot actions always
override passive shift patterns.

| Priority | State           | Condition                               | Pattern              |
|----------|-----------------|-----------------------------------------|----------------------|
| 1        | VISION_ALIGNED  | Auto-aim on AND locked on AprilTag      | Solid Green          |
| 2        | AUTO_AIM_ACTIVE | Auto-aim toggled on, still searching    | Solid Yellow         |
| 3        | INTAKING        | Intake rollers running                  | Solid Orange         |
| 4        | AUTONOMOUS      | Running an auto routine                 | Rainbow              |
| 5        | DISABLED        | Robot disabled (alliance-aware color)   | Breathing Red/Blue   |
| 6        | END_GAME        | Teleop, last 30 seconds                | Strobe Red           |
| 7        | SHIFT_1         | Teleop, 130-105s remaining             | Color Flow (alliance)|
| 8        | SHIFT_2         | Teleop, 105-80s remaining              | RGB Fade             |
| 9        | SHIFT_3         | Teleop, 80-55s remaining               | Larson Scanner Red   |
| 10       | SHIFT_4         | Teleop, 55-30s remaining               | Breathing Red        |
| 11       | IDLE            | Nothing happening, or transition period | Fire                 |

## Teleop Shift Timing

REBUILT teleop is 2:20 (140 seconds). Shifts are based on time remaining:

| Phase      | Time Remaining | What Happens                            |
|------------|----------------|-----------------------------------------|
| Transition | 140-130s       | LED shows IDLE (fire). Getting started. |
| Shift 1    | 130-105s       | Color Flow pattern (alliance color).    |
| Shift 2    | 105-80s        | RGB Fade pattern.                       |
| Shift 3    | 80-55s         | Larson Scanner pattern.                 |
| Shift 4    | 55-30s         | Breathing Red. Getting urgent.          |
| End Game   | 30-0s          | Strobe Red. Prep climb, score all.      |

These boundaries match Robot.java shift tracking.

## How State Changes Work

Animations are only sent when the state actually changes. On each state change,
all 8 animation slots are cleared first, then the new pattern is applied. This
prevents animation conflicts and ghosting between patterns.

## Alliance-Aware Patterns

When FMS connects and reports alliance color, DISABLED and SHIFT_1 patterns
use alliance color (red or blue). Before FMS connects, defaults to blue.

## Changing Patterns

All colors and animation speeds live in LEDConstants.java. To change a pattern:

1. Find the animation you want in the Phoenix 6 CANdle controls package
2. Update the applyPattern() switch case in LEDSubsystem.java
3. Add any new constants to LEDConstants.java
4. Build and deploy

## Available CANdle Animations

These are the Phoenix 6 animation classes in `com.ctre.phoenix6.controls`:

| Animation           | Description                                        |
|---------------------|----------------------------------------------------|
| SolidColor          | Static color on a range of LEDs                    |
| RainbowAnimation    | Scrolling rainbow across all LEDs                  |
| FireAnimation       | Flickering flame effect                            |
| StrobeAnimation     | Flashing strobe effect                             |
| SingleFadeAnimation | Breathing fade in/out of one color                 |
| ColorFlowAnimation  | Color fills strip one LED at a time                |
| LarsonAnimation     | KITT/Cylon scanner bounce                          |
| RgbFadeAnimation    | Smooth fade between red, green, blue               |
| TwinkleAnimation    | Random sparkle effect                              |

## Adding a New State

1. Add the enum value to LEDState in LEDSubsystem.java (position = priority)
2. Add any new colors or speeds to LEDConstants.java
3. Add the condition check in periodic() waterfall
4. Add the case to applyPattern() switch
