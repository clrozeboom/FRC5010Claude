# Driver Controls — input pipeline and button bindings

How raw joystick values become drive commands, and how to bind buttons without
tripping the classic "rotation does nothing on a real Xbox pad" gotcha.

| Class | Where | Role |
|---|---|---|
| `JoystickAxis` | `org/frc5010/common/input/JoystickAxis.java` | One axis + a chain of transforms (deadzone, power, scale, limit, negate) |
| `DriveVector` | `org/frc5010/common/input/DriveVector.java` | Two axes combined into an `(x, y)` translation, with optional unit-circle shaping |
| `ConfigurableController` | `org/frc5010/common/input/ConfigurableController.java` | Generic port-based controller — `axis(int)` / `button(int)` |
| `XboxConfigurableController` | `org/frc5010/common/input/XboxConfigurableController.java` | Named Xbox accessors — `leftY()`, `rightX()`, `a()`, … |

---

## The pipeline

```
raw axis (DoubleSupplier)
      │  JoystickAxis.of(...)
      ▼
JoystickAxis  ──negate()──deadzone()──power()──scale()──limit()──►  getAsDouble()
      │                                                                 │
      └──────────────── DriveVector.of(forwardAxis, strafeAxis) ────────┘
                                   │ unitCircle()
                                   ▼
                              Translation2d  → ChassisSpeeds → drive.runVelocityFieldRelative(...)
```

`JoystickAxis` is a `DoubleSupplier`, so the transforms compose lazily and are
re-read every loop — you build the chain once at binding time and it stays live.

### JoystickAxis transforms

| Method | Effect | Typical use |
|---|---|---|
| `negate()` | flips sign | WPILib forward is −Y on a stick |
| `deadzone(t)` | zero below `\|v\| ≤ t`, rescales the rest | kill stick drift (0.05–0.10) |
| `power(n)` | `sign(v)·\|v\|ⁿ` | finer low-speed control (2.0 is common) |
| `scale(f)` | multiply by `f` | trim a too-fast axis |
| `limit(m)` | clamp to `±m` | cap a beginner driver's top speed |

```java
JoystickAxis forward = controller.leftY().negate().deadzone(0.05).power(2.0);
```

### DriveVector

`DriveVector.of(x, y)` carries the same transforms but applies them to the pair,
and adds `unitCircle()`, which maps the square stick gate to a disc so a full
diagonal push isn't √2× faster than a straight push. Call `get()` for the
shaped `Translation2d`.

---

## The two controllers

`XboxConfigurableController` is what `SwerveRobotContainer` hands subclasses as
the `controller` field. Use the **named** accessors — they map to the correct
axes for a real Xbox pad:

| Named accessor | Underlying axis | Named accessor | Button |
|---|---|---|---|
| `leftX()` | 0 | `a()` / `b()` / `x()` / `y()` | face buttons |
| `leftY()` | 1 | `leftBumper()` / `rightBumper()` | bumpers |
| `leftTrigger()` | 2 | `back()` / `start()` | center |
| `rightTrigger()` | 3 | `leftStick()` / `rightStick()` | stick clicks |
| `rightX()` | 4 | | |
| `rightY()` | 5 | | |

`ConfigurableController` (the base) exposes raw `axis(int)` / `button(int)` for
non-Xbox devices. Prefer the named Xbox accessors whenever you can — they are
why the gotcha below exists.

---

## Rotation axis: standard Xbox layout, sim and hardware

The default drive command uses the **standard Xbox axes** — left stick translates, right-stick X
rotates:

```java
JoystickAxis forward  = controller.leftY().negate().deadzone(0.05);  // axis 1
JoystickAxis strafe   = controller.leftX().negate().deadzone(0.05);  // axis 0
JoystickAxis rotation = controller.rightX().negate().deadzone(0.05); // axis 4
```

The classic "rotation does nothing on a real pad" surprise came from the Glass keyboard mapping
rotation to **axis 2** — which on a physical Xbox pad is the **left trigger**, not rotation. The
fix is that **the Glass keyboard joystick (`simgui-ds.json`) is configured to mirror a real Xbox
controller**: 6 axes, 10 buttons, 1 POV, with `W/S → leftY`, `A/D → leftX`, **`E/Q → rightX`
(axis 4)**. So the *same* standard mapping drives the sim keyboard and a real controller — no
per-team override, no runtime axis juggling. (Pinned by `SwerveDriveAxisTest`: right-stick X
rotates, the left trigger does not.)

> If you regenerate `simgui-ds.json` from a fresh Glass session, re-apply the Xbox keyboard
> layout (axisCount 6, rotation on axis 4) or rotation will land on the wrong axis in sim.

Want a different feel (larger rotation deadzone, a `power()` curve)? Override the drive default
command in your subclass after `super.configureBindings()` with the named accessors.

---

## Button bindings

Bindings live in `configureBindings()`. **Always call `super.configureBindings()`
first** — it wires the drive default command, web control, and standard buttons.
Skipping it leaves the robot with no default drive command.

```java
controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
controller.b().onTrue(elevator.goToHeight(YourElevator.SCORING_HEIGHT));
controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));
```

### Goal commands never finish

A mechanism goal command such as `elevator.goToHeight(...)` **holds its goal
forever** (until another command grabs the same subsystem). Bind one button per
preset with `.onTrue(...)`; do **not** chain presets with `.andThen(...)`
expecting the first to release — it never will.

### Lifecycle: bind inside `configureBindings()`, not the constructor

`SwerveRobotContainer`'s constructor calls `configureBindings()` **before your
subclass constructor body runs**. Subsystems you assign in the subclass
constructor body are still `null` when `configureBindings()` executes. Construct
and bind subsystems *inside* `configureBindings()`, and register their teardown
with `registerMechanism(subsystem::close)` so the test suite frees CAN/PWM
handles between cases.

---

## See also

- [docs/robot-profiles.md](robot-profiles.md) — where the `controller`,
  `drive`, and lifecycle come from
- [docs/build-your-robot.md](build-your-robot.md) — the guided lesson plan that
  applies all of this end to end (Lesson 5)
- [docs/mechanisms.md](mechanisms.md) — the goal commands you bind buttons to
