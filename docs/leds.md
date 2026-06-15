# LED strips — segmented animations linked to robot state

## Overview

The LED stack is split the same way as the rest of the library:

| Layer | Class | Role |
|---|---|---|
| Common (game-agnostic) | `org.frc5010.common.leds.LedStripSegments` | One `AddressableLED` strip divided into independently animated segments, plus a whole-strip override |
| Common (game-agnostic) | `org.frc5010.common.leds.LedAnimations` | Custom time-based `LEDPattern`s WPILib doesn't ship (Larson scanner, laser bolt) |
| Team code (example) | `org.frc5010.examples.DemoLeds` | Maps 2026 demo-robot states (alliance, enabled/disabled, intake, shooting) onto a 30-LED strip |

WPILib supports **one `AddressableLED` driver per robot**, so create a single
`LedStripSegments` and divide it into as many segments as you need.

## Library API (`LedStripSegments`)

```java
LedStripSegments strip = new LedStripSegments(9, 30);          // PWM port, LED count
LedStripSegments.Segment left   = strip.addSegment(0, 9, true); // reversed (renders back-to-front)
LedStripSegments.Segment middle = strip.addSegment(10, 19);
LedStripSegments.Segment right  = strip.addSegment(20, 29);

middle.setPattern(LEDPattern.solid(Color.kGreen));              // any WPILib LEDPattern
left.setPattern(LedAnimations.laser(Color.kBlue, Seconds.of(0.4), 4));
```

- Each segment holds a `LEDPattern` that is re-applied every `periodic()`, so
  time-based patterns animate automatically. Patterns are pure functions of the
  FPGA timestamp — stateless, shareable between segments, and deterministic under
  the stepped sim clock.
- `setOverride(pattern)` renders one pattern across the whole strip, taking
  precedence over all segments (for global states like "disabled"); segment
  patterns are retained and resume after `clearOverride()`.
- A **reversed** segment renders its pattern back-to-front — use it to make
  directional effects (laser, scroll) on the two halves of a strip travel
  symmetrically outward from the centre.
- `LedStripSegments` is `AutoCloseable`: tests and `closeMechanisms()` free the
  PWM port so a later robot construction can re-create the strip.

## Custom animations (`LedAnimations`)

| Factory | Effect |
|---|---|
| `larson(color, period, tailLength)` | Knight Rider / Cylon: a bright eye sweeping end-to-end with a linear falloff tail |
| `laser(color, period, pulseLength)` | A bright bolt with a fading tail that repeatedly travels from the segment start off its far end |

WPILib's built-ins cover the rest (`LEDPattern.solid`, `LEDPattern.rainbow(...)
.scrollAtRelativeSpeed(Hertz.of(0.5))`, blink, gradient, …).

## Demo state mapping (`DemoLeds`, 30 LEDs)

Segments: left side 0–9 (reversed), middle 10–19, right side 20–29.

| Robot state | Strip behaviour |
|---|---|
| Disabled, never yet enabled (startup) | Whole strip solid alliance colour once `DriverStation.getAlliance()` reports one (re-checked every cycle, so a late DS connection updates it); solid green until then |
| Disabled after having been enabled | Scrolling rainbow across the whole strip |
| Enabled, intake deployed | Red Larson scanner on the middle segment |
| Enabled, intake retracted | Solid green on the middle segment |
| Enabled, shooting (≈1 s after each shot) | Outward laser bolts on both sides — alliance colour inside the alliance zone (`DemoIntake.isInAllianceZone`), green outside it |
| Enabled, not shooting | Sides solid alliance colour (green if unknown) |

"Shooting" is signalled by `DemoLeds.notifyShot()`, which `ExampleRobot` calls
alongside `DemoIntake.fireCommand()` on the A button (only when fuel is actually
held). The effect runs on a 1 s timer rather than a command so it never competes
for the drive or intake subsystems.

On a real robot, construct the strip unconditionally (outside the sim guard) and
feed it real mechanism state; in `ExampleRobot` it lives with the sim-only
`DemoIntake` because that is where its states come from. Register
`leds::close` via `registerMechanism(...)` so tests can free the PWM port.

## Web UI display

When the sim web UI is active (`./gradlew simulateJava -PwebUI`), `LedStripSegments`'s
constructor auto-binds the strip to `WebControl` (same pattern as `SimRobotState`), and
the browser renders a live copy of it as a row of glowing dots **under the field view**.

- The colour snapshot is taken every cycle on the robot thread in
  `WebDriveController.applyPendingControl()` — the always-running hook (gotcha 11), so
  the display stays live in disabled/auto/teleop alike.
- `/api/state` serves it as `"leds":["#rrggbb", …]`; the bar is hidden while the array
  is empty (no strip constructed).
- `WebUIFunctionalTest.ledStripSurfacesAndAnimatesInStateJson` (Layer 4) asserts the
  array has all 30 colours and animates between polls.

## Tests

- `org.frc5010.common.subsystem.LedStripSegmentsTest` — Layer 2: segment
  independence, override precedence/restore, reversed mapping, default-off.
- `org.frc5010.examples.DemoLedsTest` — Layer 2: every state → pattern mapping
  above, asserted by reading rendered colours per segment from the buffer.
