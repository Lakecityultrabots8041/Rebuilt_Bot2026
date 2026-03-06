# LED Subsystem Guide

## How the Blinkin Works

The Rev Blinkin is controlled like a Spark motor controller over PWM.
Sending a value between -1.0 and 1.0 selects a built-in pattern.
Check the printed card that shipped with your Blinkin for the full pattern list.

Hardware is on PWM port 0 (configured in LEDConstants.java).

## Priority Waterfall

The LED picks one pattern per loop cycle using a first-match-wins priority system.
The highest-priority condition that is true wins. Active robot actions always
override passive shift patterns.

| Priority | State           | Condition                               | Pattern                |
|----------|-----------------|-----------------------------------------|------------------------|
| 1        | VISION_ALIGNED  | Auto-aim on AND locked on AprilTag      | Solid Green (0.75)     |
| 2        | AUTO_AIM_ACTIVE | Auto-aim toggled on, still searching    | Solid Yellow (0.67)    |
| 3        | INTAKING        | Intake rollers running                  | Solid Orange (0.63)    |
| 4        | AUTONOMOUS      | Running an auto routine                 | Rainbow (-0.99)        |
| 5        | DISABLED        | Robot disabled (alliance-aware color)   | Breath Red/Blue        |
| 6        | END_GAME        | Teleop, last 30 seconds                | Strobe Red (-0.11)     |
| 7        | SHIFT_1         | Teleop, 130-105s remaining             | Color Waves (-0.45)    |
| 8        | SHIFT_2         | Teleop, 105-80s remaining              | BPM Rainbow (-0.69)    |
| 9        | SHIFT_3         | Teleop, 80-55s remaining               | Larson Scanner (-0.35) |
| 10       | SHIFT_4         | Teleop, 55-30s remaining               | Heartbeat Red (-0.25)  |
| 11       | IDLE            | Nothing happening, or Transition period | Fire Large (-0.57)     |

## Teleop Shift Timing

REBUILT teleop is 2:20 (140 seconds). Shifts are based on time remaining:

| Phase      | Time Remaining | What Happens                            |
|------------|----------------|-----------------------------------------|
| Transition | 140-130s       | LED shows IDLE (fire). Getting started. |
| Shift 1    | 130-105s       | Color Waves pattern.                    |
| Shift 2    | 105-80s        | BPM pattern.                            |
| Shift 3    | 80-55s         | Larson Scanner pattern.                 |
| Shift 4    | 55-30s         | Heartbeat pattern. Getting urgent.      |
| End Game   | 30-0s          | Strobe. Prep climb, score everything.   |

These boundaries match Robot.java shift tracking. The dashboard also shows
"Driver/Next Shift In" (countdown) and "Driver/Shift Warning" (true at 4s before change).

## How State Changes Work

PWM is only sent when the state actually changes. This avoids redundant writes every
loop cycle. When an action starts (like intaking), the LED switches to that action's
pattern. When the action stops, it falls back to the current shift pattern automatically
because the waterfall re-evaluates and the shift condition is now the highest match.

## Alliance-Aware Disabled Pattern

When the FMS connects and reports alliance color, the disabled pattern switches
to Breath Red or Breath Blue. If FMS has not connected yet, it defaults to Breath Blue.
If the alliance changes (re-plugging into a different station), the pattern updates
on the next loop cycle.

## Changing Patterns

All PWM values live in LEDConstants.java. To swap a pattern:

1. Find the Spark value you want from the reference table below
2. Update the constant in LEDConstants.java
3. Build and deploy

No changes needed in LEDSubsystem.java unless you are adding or removing states.

## Adding a New State

1. Add the enum value to LEDState in LEDSubsystem.java (position = priority)
2. Add the PWM constant to LEDConstants.java
3. Add the condition check in periodic() waterfall
4. Add the case to getPatternValue() switch

## Rev Blinkin Pattern Reference

Source: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf

The Spark value is what you pass to `blinkin.set()` in code. Each value maps to
a built-in pattern on the Blinkin hardware. Valid range is -1.0 to 1.0.

### Fixed Palette Patterns

| #  | PWM (us) | Spark Value | Pattern                            |
|----|----------|-------------|------------------------------------|
| 1  | 1005     | -0.99       | Rainbow, Rainbow Palette           |
| 2  | 1015     | -0.97       | Rainbow, Party Palette             |
| 3  | 1025     | -0.95       | Rainbow, Ocean Palette             |
| 4  | 1035     | -0.93       | Rainbow, Lava Palette              |
| 5  | 1045     | -0.91       | Rainbow, Forest Palette            |
| 6  | 1055     | -0.89       | Rainbow with Glitter               |
| 7  | 1065     | -0.87       | Confetti                           |
| 8  | 1075     | -0.85       | Shot, Red                          |
| 9  | 1085     | -0.83       | Shot, Blue                         |
| 10 | 1095     | -0.81       | Shot, White                        |
| 11 | 1105     | -0.79       | Sinelon, Rainbow Palette           |
| 12 | 1115     | -0.77       | Sinelon, Party Palette             |
| 13 | 1125     | -0.75       | Sinelon, Ocean Palette             |
| 14 | 1135     | -0.73       | Sinelon, Lava Palette              |
| 15 | 1145     | -0.71       | Sinelon, Forest Palette            |
| 16 | 1155     | -0.69       | Beats per Minute, Rainbow Palette  |
| 17 | 1165     | -0.67       | Beats per Minute, Party Palette    |
| 18 | 1175     | -0.65       | Beats per Minute, Ocean Palette    |
| 19 | 1185     | -0.63       | Beats per Minute, Lava Palette     |
| 20 | 1195     | -0.61       | Beats per Minute, Forest Palette   |
| 21 | 1205     | -0.59       | Fire, Medium                       |
| 22 | 1215     | -0.57       | Fire, Large                        |
| 23 | 1225     | -0.55       | Twinkles, Rainbow Palette          |
| 24 | 1235     | -0.53       | Twinkles, Party Palette            |
| 25 | 1245     | -0.51       | Twinkles, Ocean Palette            |
| 26 | 1255     | -0.49       | Twinkles, Lava Palette             |
| 27 | 1265     | -0.47       | Twinkles, Forest Palette           |
| 28 | 1275     | -0.45       | Color Waves, Rainbow Palette       |
| 29 | 1285     | -0.43       | Color Waves, Party Palette         |
| 30 | 1295     | -0.41       | Color Waves, Ocean Palette         |
| 31 | 1305     | -0.39       | Color Waves, Lava Palette          |
| 32 | 1315     | -0.37       | Color Waves, Forest Palette        |
| 33 | 1325     | -0.35       | Larson Scanner, Red                |
| 34 | 1335     | -0.33       | Larson Scanner, Gray               |
| 35 | 1345     | -0.31       | Light Chase, Red                   |
| 36 | 1355     | -0.29       | Light Chase, Blue                  |
| 37 | 1365     | -0.27       | Light Chase, Gray                  |

### Fixed Palette Heartbeat, Breath, Strobe

| #  | PWM (us) | Spark Value | Pattern                            |
|----|----------|-------------|------------------------------------|
| 38 | 1375     | -0.25       | Heartbeat, Red                     |
| 39 | 1385     | -0.23       | Heartbeat, Blue                    |
| 40 | 1395     | -0.21       | Heartbeat, White                   |
| 41 | 1405     | -0.19       | Heartbeat, Gray                    |
| 42 | 1415     | -0.17       | Breath, Red                        |
| 43 | 1425     | -0.15       | Breath, Blue                       |
| 44 | 1435     | -0.13       | Breath, Gray                       |
| 45 | 1445     | -0.11       | Strobe, Red                        |
| 46 | 1455     | -0.09       | Strobe, Blue                       |
| 47 | 1465     | -0.07       | Strobe, Gold                       |
| 48 | 1475     | -0.05       | Strobe, White                      |

### Color 1 Patterns

Color 1 is set with the potentiometer on the Blinkin.

| #  | PWM (us) | Spark Value | Pattern                            |
|----|----------|-------------|------------------------------------|
| 49 | 1485     | -0.03       | End to End Blend to Black          |
| 50 | 1495     | -0.01       | Larson Scanner                     |
| 51 | 1505     | 0.01        | Light Chase                        |
| 52 | 1515     | 0.03        | Heartbeat Slow                     |
| 53 | 1525     | 0.05        | Heartbeat Medium                   |
| 54 | 1535     | 0.07        | Heartbeat Fast                     |
| 55 | 1545     | 0.09        | Breath Slow                        |
| 56 | 1555     | 0.11        | Breath Fast                        |
| 57 | 1565     | 0.13        | Shot                               |
| 58 | 1575     | 0.15        | Strobe                             |

### Color 2 Patterns

Color 2 is set with the potentiometer on the Blinkin.

| #  | PWM (us) | Spark Value | Pattern                            |
|----|----------|-------------|------------------------------------|
| 59 | 1585     | 0.17        | End to End Blend to Black          |
| 60 | 1595     | 0.19        | Larson Scanner                     |
| 61 | 1605     | 0.21        | Light Chase                        |
| 62 | 1615     | 0.23        | Heartbeat Slow                     |
| 63 | 1625     | 0.25        | Heartbeat Medium                   |
| 64 | 1635     | 0.27        | Heartbeat Fast                     |
| 65 | 1645     | 0.29        | Breath Slow                        |
| 66 | 1655     | 0.31        | Breath Fast                        |
| 67 | 1665     | 0.33        | Shot                               |
| 68 | 1675     | 0.35        | Strobe                             |

### Color 1 and 2 Combined

| #  | PWM (us) | Spark Value | Pattern                            |
|----|----------|-------------|------------------------------------|
| 69 | 1685     | 0.37        | Sparkle, Color 1 on Color 2       |
| 70 | 1695     | 0.39        | Sparkle, Color 2 on Color 1       |
| 71 | 1705     | 0.41        | Color Gradient, Color 1 and 2     |
| 72 | 1715     | 0.43        | Beats per Minute, Color 1 and 2   |
| 73 | 1725     | 0.45        | End to End Blend, Color 1 to 2    |
| 74 | 1735     | 0.47        | End to End Blend                   |
| 75 | 1745     | 0.49        | Color 1 and 2 no blending (Setup) |
| 76 | 1755     | 0.51        | Twinkles, Color 1 and 2           |
| 77 | 1765     | 0.53        | Color Waves, Color 1 and 2        |
| 78 | 1785     | 0.57        | Sinelon, Color 1 and 2            |

### Solid Colors

| #   | PWM (us) | Spark Value | Pattern                            |
|-----|----------|-------------|------------------------------------|
| 79  | 1785     | 0.57        | Hot Pink                           |
| 80  | 1795     | 0.59        | Dark Red                           |
| 81  | 1805     | 0.61        | Red                                |
| 82  | 1815     | 0.63        | Red Orange                         |
| 83  | 1825     | 0.65        | Orange                             |
| 84  | 1835     | 0.67        | Gold                               |
| 85  | 1845     | 0.69        | Yellow                             |
| 86  | 1855     | 0.71        | Lawn Green                         |
| 87  | 1865     | 0.73        | Lime                               |
| 88  | 1875     | 0.75        | Dark Green                         |
| 89  | 1885     | 0.77        | Green                              |
| 90  | 1895     | 0.79        | Blue Green                         |
| 91  | 1905     | 0.81        | Aqua                               |
| 92  | 1915     | 0.83        | Sky Blue                           |
| 93  | 1925     | 0.85        | Dark Blue                          |
| 94  | 1935     | 0.87        | Blue                               |
| 95  | 1945     | 0.89        | Blue Violet                        |
| 96  | 1955     | 0.91        | Violet                             |
| 97  | 1965     | 0.93        | White                              |
| 98  | 1975     | 0.95        | Gray                               |
| 99  | 1985     | 0.97        | Dark Gray                          |
| 100 | 1995     | 0.99        | Black                              |
