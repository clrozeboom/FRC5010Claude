# Build Your Robot — a guided lesson plan

> **Format:** This is a guided walkthrough. Each lesson has a goal, the concepts
> you need to understand *before* writing code, the work itself, a checkpoint to
> verify before moving on, and links to the **deep-dive docs** that cover each
> topic in full. Tell Claude when you hit an issue at a checkpoint and it will
> revise the lesson before continuing.

> **Naming:** this plan uses the placeholder robot name **`Comp`** (for "your
> competition robot") and the package **`frc.robot.comp`**. Replace both with
> your robot's name everywhere — e.g. `frc.robot.viper` / `ViperRobot`.
>
> **Worked solution:** the [`tigershark`](https://github.com/clrozeboom/FRC5010Claude/tree/tigershark)
> branch is a complete, working example of this exact plan (a Kraken swerve plus
> a two-motor elevator). Peek there if a lesson leaves you stuck.

## Course overview

**You are building:** a new FRC robot configuration — a swerve drivetrain
(Krakens via Phoenix Tuner X) plus a two-motor elevator on a single gearbox.
Vision is left disabled until a coprocessor is connected. LEDs respond to robot
state through generic suppliers.

**What you'll learn along the way:**

- How the framework separates **hardware constants** (a `RobotProfile`) from
  **robot behaviour** (a `SwerveRobotContainer` subclass) from **the wiring
  shell** (`RobotContainer.java`).
- How TalonFX swerve REAL-mode wiring differs from sim wiring, and why
  `SwerveFactory.build()` deliberately throws on the hardware path.
- How a two-motor mechanism is configured (one extra field — with two gotchas).
- How LQR vs. profiled-PID control choices interact with what you actually know
  about the mechanism.
- How button bindings flow through `configureBindings()` and why subsystem
  lifecycle (`registerMechanism`) matters for tests.

**Course structure:**

| # | Lesson | Goal | Deep dives |
|---|---|---|---|
| 0 | Branch setup | Isolate your work from `main` | [environment](environment.md) |
| 1 | TunerConstants intake | Understand what TunerX produces | [configuration](configuration.md) |
| 2 | RobotProfile | Describe the drivetrain to the framework | [robot-profiles](robot-profiles.md), [configuration](configuration.md), [vision](vision.md) · `/new-robot-profile` |
| 3 | Elevator | Configure the two-motor elevator | [mechanisms](mechanisms.md) · `/new-mechanism` |
| 4 | LEDs | Map robot state to LED patterns | [leds](leds.md) |
| 5 | Robot container | Wire subsystems into button bindings | [controls](controls.md), [robot-profiles](robot-profiles.md) |
| 6 | RobotContainer swap | Point the entry point at your robot | [architecture](architecture.md) |
| 7 | Sim verification | Prove it works before hardware | [testing](testing.md), [simulation](simulation.md) · `/new-sim-test` |
| 8 | Hand-off | Tuning, calibration, deploy | [calibration](calibration.md), [mechanisms](mechanisms.md), [auto](auto.md), [student-setup](student-setup.md) |

**Files you'll create** (all under `src/main/java/frc/robot/comp/`):
`TunerConstants.java`, `CompRobotProfile.java`, `CompElevator.java`,
`CompLeds.java`, `CompRobot.java`.

**Files you'll edit:** just `src/main/java/frc/robot/RobotContainer.java`
(three lines).

**Files you'll leave alone but use as reference:** everything in
`org/frc5010/examples/` and `org/frc5010/examples/mechanisms/Example*.java`.

> **Running commands — platform note.** Examples below show `./gradlew`
> (macOS / Linux / Codespaces). On Windows local, use `.\gradlew.bat` from
> PowerShell. See [docs/environment.md](environment.md).

---

## Lesson 0 — Branch setup

### Goal
Get onto a feature branch so your robot's code can evolve separately from the
upstream library.

### Concept
The `main` branch of this repo is the **library**. Your robot is downstream of
it. Doing your team's work on a branch means you can keep pulling library
improvements without merge fights and without polluting `main` with
team-specific constants. → [docs/environment.md](environment.md)

### Work
```bash
git checkout -b comp        # your robot's name
```

### Checkpoint
- `git status` shows your branch, working tree clean.
- `git log --oneline -1` shows the same commit that's at the tip of `main`.

### Self-check question
*If your team later wants to test this code on a second physical chassis, would
you branch off your robot branch, or off `main`? Why?*

---

## Lesson 1 — TunerConstants intake

### Goal
Get the Phoenix Tuner X export into the project as untouchable generated code.

### Concept
Phoenix Tuner X's Swerve Project Generator produces **one file**,
`TunerConstants.java`, containing the four `SwerveModuleConstants`
(`FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`), all drive/steer/CANcoder
IDs, the gyro ID, the CAN bus name, wheel radius, gear ratios, and
TunerX-calibrated gains.

Why "untouchable": when you re-tune (new offsets after a crash, a new gearbox,
new wheels) you re-run TunerX and **overwrite** this file. Hand-edits get wiped.
Put your hand-edits in the `RobotProfile` (Lesson 2) instead.

→ For how those numbers feed the framework, see
[docs/configuration.md](configuration.md).

### Work
1. Paste the TunerX-generated file into
   `src/main/java/frc/robot/comp/TunerConstants.java`.
2. Change the package declaration to `package frc.robot.comp;`.
3. **Read it once.** Note these names — you'll cite them in Lesson 2:
   `kFrontLeftDriveMotorId`, `kFrontLeftSteerMotorId`, `kFrontLeftEncoderId`,
   `kFrontLeftEncoderOffset`, the wheel-radius constant, `kDriveGearRatio`,
   `kSteerGearRatio`, `kCANBus`, and the four module objects.

### Checkpoint
- The file compiles in your IDE.
- You can answer the questions below from reading it.

### Self-check questions
- *What CAN ID is your Front-Left drive motor on?*
- *Is `kCANBus` set to `""` (RIO bus) or to a CANivore device name?*
- *What's the wheel-radius constant, in inches and metres?*

---

## Lesson 2 — RobotProfile

### Goal
Describe the swerve drivetrain to the framework: physical geometry, CAN IDs,
gyro choice, starting pose, and the REAL/SIM dispatch in `createDrive()`.

> **Deep dives:** [docs/robot-profiles.md](robot-profiles.md) (the pattern,
> REAL/SIM branching, field length, vision wiring) ·
> [docs/configuration.md](configuration.md) (every `SwerveConstants` field) ·
> [docs/vision.md](vision.md) (when you add a camera). The
> **`/new-robot-profile`** slash command is the step-by-step version of this
> lesson.

### Concepts

**Three reasons `SwerveConstants` exists:**

1. **REAL mode** uses it to dispatch to the right `ModuleIO` (TalonFX vs.
   Spark/Talon) and the right gyro class.
2. **SIM mode** uses it to build an IronMaple physics model of *your specific
   robot*. Mass, bumper size, and wheel geometry shape how the sim accelerates
   and collides. Lie about the mass → sim is fast, real robot is slow.
3. **Motion control** uses `maxLinearSpeed` etc. to clamp teleop and plan
   trajectories.

**Why the factory throws in REAL mode (CLAUDE.md gotcha #8):**
`SwerveFactory.build()` can wire up sim-mode TalonFX because the sim path uses
synthesized motor specs. The real path needs your *team's*
`SwerveModuleConstants` — gear ratios, gains, encoder offsets — which it can't
guess. So it throws, and your `createDrive()` wires the IO classes manually. The
throw isn't a bug; it's a "you need to do this part" sign.

**Starting pose:** write only the blue-alliance start. The framework mirrors it
for red automatically (`SwerveRobotContainer.getAllianceStartPose()`).

### Work
Path: `src/main/java/frc/robot/comp/CompRobotProfile.java`

1. Copy `org/frc5010/examples/ExampleRobotProfile.java` to the new location;
   change the package to `frc.robot.comp` and rename the class.

2. Rewrite the `CONSTANTS` builder with your real numbers (full field reference:
   [docs/configuration.md](configuration.md)):

   ```java
   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
       .moduleType(ModuleType.TALON_FX)
       .gyroType(GyroType.PIGEON2)             // or NAVX
       .gyroCanId(/* TunerConstants.kPigeonId */)
       .trackWidth(Inches.of(/* MEASURED left-to-right wheel centres */))
       .wheelBase(Inches.of(/* MEASURED front-to-back wheel centres */))
       .wheelRadius(Inches.of(/* TunerX worn-wheel value */))
       .maxLinearSpeed(MetersPerSecond.of(/* measured top speed */))
       .maxAngularSpeed(RadiansPerSecond.of(/* measured top spin rate */))
       .robotMass(Pounds.of(/* weighed with bumpers + battery */))
       .bumperLength(Inches.of(/* outside-to-outside, front-to-back */))
       .bumperWidth(Inches.of(/* outside-to-outside, side-to-side */))
       .frontLeftIds(/* drive, steer, encoder — from TunerConstants */)
       .frontRightIds(/* … */)
       .backLeftIds(/* … */)
       .backRightIds(/* … */)
       .canBusName("")                         // match TunerConstants.kCANBus
       .odometryFrequency(Hertz.of(100))       // 250 for CANivore, 100 for RIO bus
       .build();
   ```

   **Why duplicate the CAN IDs here AND in `TunerConstants`?** The framework
   needs them for sim-mode IO selection independent of the TunerX record. If
   they disagree, sim and real diverge silently — make them match.

3. Set `BLUE_START` to your team's blue-alliance starting position.

4. Replace the `throw new UnsupportedOperationException(...)` in `createDrive()`
   with the real wiring:

   ```java
   if (RobotBase.isReal()) {
     GyroIO gyro = new GyroIOPigeon2(CONSTANTS);
     ModuleIO[] modules = {
       new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontLeft),
       new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontRight),
       new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackLeft),
       new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackRight),
     };
     return new AkitSwerveDrive(CONSTANTS, gyro, modules);
   }
   return SwerveFactory.build(CONSTANTS, BLUE_START);
   ```

   Add imports for `GyroIO`, `GyroIOPigeon2`, `ModuleIO`, `ModuleIOTalonFXReal`,
   and `frc.robot.comp.TunerConstants`.

5. **Skip `createVision()` for now.** The base `RobotProfile.createVision()`
   returns `null`, which disables vision. Do **not** override it until a
   PhotonVision coprocessor is physically connected and configured — a missing
   camera spams `PhotonVision coprocessor not found` every loop. When you're
   ready, follow [docs/vision.md](vision.md) or **`/new-vision-camera`**.

### Checkpoint
- File compiles; no red squigglies.
- CAN IDs in `frontLeftIds(...)` match `TunerConstants.kFrontLeft*` exactly.
- `BLUE_START` is a real field position, not the placeholder `(1.5, 2.0)`.
- `createVision()` is **not** overridden.

### Self-check questions
- *Why does the simulator need `robotMass`? What goes wrong at half its value?*
- *If you swapped front-left and back-right `*Ids(...)`, what would the sim show
  when you push forward?*
- *Why is `wheelRadius` measured rather than taken from the spec sheet?*

---

## Lesson 3 — Elevator

### Goal
Describe the two-motor elevator: physics, geometry, and control style. You won't
write control logic — that's all in the common `Elevator` class.

> **Deep dive:** [docs/mechanisms.md](mechanisms.md) — TalonFX-native LQR vs.
> MotionMagic, the plant rules, every gotcha. The **`/new-mechanism`** command
> scaffolds a mechanism step by step; **`/tune-mechanism`** tunes it later.

### Concepts

**Master + follower on one gearbox.** Two motors, one drum, one output. Two
fields in `Settings`:
- `canId` — the lead motor.
- `followerCanId` — the follower (configured as a Phoenix follower of the lead).

**The plant gotcha — `motorModel`.** The simulated physics model defaults to
`DCMotor.getKrakenX60(1)`. Two motors mechanically combined ⇒ set
`DCMotor.getKrakenX60(2)`. The `2` doubles torque and current in the plant.
Forget it and sim thinks your elevator is half as strong, your LQR plant is
wrong, and your tuning is wrong.

**`followerOpposed`** — `true` when the follower faces physically opposite the
lead (common with back-to-back gearboxes). It flips the follower's voltage so
they drive the same mechanical direction. Get it wrong and the motors fight and
trip the breaker.

**Profile cruise-velocity sanity check (gotcha #12).** If `maxVelocity` exceeds
what the motors can reach, the profile runs away from the mechanism, error
grows, and the controller saturates with overshoot. Check:
```
freeSpeedRotorRps   = 100              (Kraken X60)
freeSpeedDrumRps    = 100 / gearing
freeSpeedLinearMps  = freeSpeedDrumRps × drumCircumference (m)
```
Pick `maxVelocity` ≲ 80% of `freeSpeedLinearMps`.

**Control style** ([docs/mechanisms.md](mechanisms.md) has the full comparison):
- `ControlStyle.LQR` (default): controller built from physical parameters. Best
  when you trust mass/gearing. No `kP/kI/kD` — you tune LQR weights
  `qelms`/`relms`.
- `ControlStyle.PROFILED_PID`: a trapezoid profile feeding MotionMagic. You
  provide `kP`, `kV`, optionally `kI`/`kD`. Pick when mass is uncertain or you
  want hand-tuneable gains.

Start with **LQR** — you know your mass and gearing, and the example elevator is
LQR so you can mirror its structure.

**`kG` (gravity feedforward):** volts to hold the carriage against gravity. Sim
works from any reasonable guess; characterize it on the real robot with
`sysId()` (**`/tune-mechanism`**).

### Work
Path: `src/main/java/frc/robot/comp/CompElevator.java`

1. Copy `org/frc5010/examples/mechanisms/ExampleElevator.java` into the new
   package; change the package and rename the class.

2. Rewrite `settings()`:
   ```java
   var s = new Settings();
   s.name = "CompElevator";
   s.canId = /* lead Kraken CAN ID */;
   s.followerCanId = /* follower Kraken CAN ID */;
   s.followerOpposed = /* true if mounted facing opposite */;
   s.followerVisualOffset = new Translation3d(0, /* metres to the side */, 0);

   s.motorModel = DCMotor.getKrakenX60(2);                  // ← CRITICAL: two motors
   s.gearReductionStages = new double[] { /* e.g. 5.0, 4.0 = 20:1 */ };
   s.drumCircumference = Inches.of(/* pitch_inches * teeth, or 2*pi*r */);
   s.carriageMass = Kilograms.of(/* moving mass including end effector */);

   s.minHeight = Meters.of(0);
   s.maxHeight = Meters.of(/* travel */);
   s.startingHeight = Meters.of(/* power-on position */);

   s.maxVelocity = MetersPerSecond.of(/* see sanity check above */);
   s.maxAcceleration = MetersPerSecondPerSecond.of(/* start ~2.0 */);

   s.kG = Volts.of(/* initial guess, refine with sysId later */);

   s.visualPose3d = new Pose3d(/* lead-motor mount in robot frame, m */, Rotation3d.kZero);
   return s;
   ```

3. **Run the cruise-velocity sanity check** and write the three numbers
   (`freeSpeedDrumRps`, `freeSpeedLinearMps`, your `maxVelocity`) in a comment
   above `s.maxVelocity`.

4. **Expose an `isMoving()` helper** for the LED class. The base `Elevator`
   publishes `getVelocity()`:
   ```java
   private static final double MOVING_THRESHOLD_MPS = 0.02;

   /** True when the carriage is moving fast enough to count as "in motion". */
   public boolean isMoving() {
     return Math.abs(getVelocity().in(MetersPerSecond)) > MOVING_THRESHOLD_MPS;
   }
   ```
   A 2 cm/s dead-band keeps the "moving" LED honest against gear lash and sensor
   noise. Also expose a public `SCORING_HEIGHT` constant — `CompRobot` binds a
   button to it.

### Checkpoint
- File compiles.
- `s.motorModel` is `DCMotor.getKrakenX60(2)` — not `(1)`.
- The cruise-velocity sanity check is documented in a comment.
- `isMoving()` and `SCORING_HEIGHT` are both public.

### Self-check questions
- *If `followerOpposed` is wrong, what symptom appears at power-on?*
- *If `motorModel` is `(1)` but you have two motors, what happens to LQR tuning
  on the real robot?*
- *Compute: gearing 20:1, drum circumference 3.456" (22-tooth #25 chain). What's
  the max possible carriage speed, and a safe `maxVelocity`?*

---

## Lesson 4 — LEDs

### Goal
Build LEDs that respond to *your* robot's state through generic suppliers —
decoupled from any specific subsystem.

> **Deep dive:** [docs/leds.md](leds.md) — segments, custom animations (Larson,
> laser), and the robot-state mapping pattern.

### Concept
The demo `DemoLeds` couples directly to `DemoIntake`. Re-implement using generic
`Supplier`/`BooleanSupplier` parameters so the LED class knows only about LED
state, not which subsystem produced it. The base `LedStripSegments` owns the
`AddressableLED`, splits the strip into named `Segment`s, and supports a
whole-strip override (for disabled animations). You write a thin `periodic()`
mapping `(robot state) → (LED pattern)`.

### Work
Path: `src/main/java/frc/robot/comp/CompLeds.java`

1. Copy `org/frc5010/examples/DemoLeds.java`; change the package and rename.
2. **Rip out everything referencing `DemoIntake`** — the `intakeExtended` param,
   `notifyShot()`, the `isInAllianceZone` call.
3. A reasonable starter mapping:

   | Robot state | Pattern |
   |---|---|
   | Disabled, never enabled | Solid alliance colour (green if unknown) |
   | Disabled, previously enabled | Scrolling rainbow |
   | Enabled, elevator moving | Red Larson scanner across the strip |
   | Enabled, elevator idle | Solid alliance colour |

4. Accept state as constructor args, e.g. `BooleanSupplier elevatorMoving`.
   Don't reach into the elevator from inside the LED class — that's how coupling
   grows back. The robot container passes `() -> elevator.isMoving()`.
5. Keep the `periodic()` skeleton:
   ```java
   if (DriverStation.isEnabled()) everEnabled = true;
   if (DriverStation.isDisabled()) {
       setOverride(everEnabled ? RAINBOW : allianceSolid());
   } else {
       clearOverride();
       // map state → pattern here
   }
   super.periodic();
   ```

### Checkpoint
- File compiles.
- No references to `DemoIntake`, `Fuel`, or any team-specific subsystem.
- All inputs arrive through the constructor as `Supplier`/`BooleanSupplier`.

### Self-check question
- *Why pass `BooleanSupplier elevatorMoving` instead of the full elevator
  reference? What's the downside of handing the LED class the whole subsystem?*

---

## Lesson 5 — Robot container

### Goal
Wire the drive (built by the profile), the elevator, and the LEDs into a
`SwerveRobotContainer` subclass, and bind controller buttons.

> **Deep dives:** [docs/controls.md](controls.md) — the input pipeline, button
> bindings, and the **`axis(2)` rotation gotcha** you must fix here ·
> [docs/robot-profiles.md](robot-profiles.md) — the container lifecycle.

### Concepts

**`SwerveRobotContainer` lifecycle:**
1. Its constructor takes the `RobotProfile` (built via `selectProfile(FQN)`).
2. The constructor calls `configureBindings()` **before your subclass
   constructor body runs** — so initialise subsystems *inside*
   `configureBindings()`, or fields read there will be `null`.
3. `buildAutos()` runs lazily on the first scheduler tick, after construction —
   safe for any subsystem reference.
4. `registerMechanism(closeable)` registers teardown so the smoke test frees CAN
   IDs and PWM handles between tests. Skipping it makes the suite flaky.

**Goal commands never finish** — `elevator.goToHeight(...)` holds forever. Bind
one `.onTrue(...)` button per preset; don't `.andThen` them.

**`selectProfile(...)` is reflection** — you pass the fully-qualified class name
as a string. Typos fail at runtime, not compile time.

**The `axis(2)` gotcha** — the base default drive maps rotation to `axis(2)`,
which is the **left trigger** on a real Xbox pad. You must override the drive
default command after `super.configureBindings()` using `controller.rightX()`.
Full explanation and the exact code: [docs/controls.md](controls.md).

### Work
Path: `src/main/java/frc/robot/comp/CompRobot.java`

1. Copy `org/frc5010/examples/ExampleRobot.java` into the new package; rename
   the class.
2. **Strip out** the demo-mechanism construction (`configureDemoMechanisms()`,
   `demoElevator`, the `Example*` imports, `DemoIntake`, the X-button binding).
   Keep the class shape (`extends SwerveRobotContainer`, `buildAutos()`,
   `configureBindings()`).
3. Pass your profile's FQN to `super`:
   ```java
   public CompRobot() {
     super(SwerveRobotContainer.selectProfile("frc.robot.comp.CompRobotProfile"));
   }
   ```
4. Stub `buildAutos()` (real autos later — [docs/auto.md](auto.md)):
   ```java
   @Override protected void buildAutos() { addAuto("None", Commands.none()); }
   ```
5. Write `configureBindings()` — drive override + elevator + LEDs. The full
   drive-override block (the `axis(2)` fix) is in
   [docs/controls.md](controls.md); the binding shape is:
   ```java
   @Override
   protected void configureBindings() {
     super.configureBindings();          // wires drive + standard buttons — don't skip

     // Override the default drive with proper Xbox axes — see docs/controls.md
     // (rotation must come from controller.rightX(), NOT axis(2)).
     // ... drive.setDefaultCommand(...) ...

     elevator = new CompElevator();
     registerMechanism(elevator::close);
     controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
     controller.b().onTrue(elevator.goToHeight(CompElevator.SCORING_HEIGHT));
     controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));

     leds = new CompLeds(LED_PWM_PORT, elevator::isMoving);
     registerMechanism(leds::close);
   }
   ```
6. Declare the fields and PWM port:
   ```java
   private static final int LED_PWM_PORT = 9;
   private CompElevator elevator;
   private CompLeds leds;
   ```

### Checkpoint
- File compiles.
- `super.configureBindings()` is the first line of `configureBindings()`.
- The drive default command uses `controller.leftY/leftX/rightX` — **not**
  `axis(2)` for rotation.
- Both `elevator` and `leds` are registered via `registerMechanism(...)`.
- No references to `Example*`, `DemoIntake`, or `DemoLeds`.

### Self-check questions
- *Why must subsystems be initialised inside `configureBindings()`?*
- *What happens if you skip `super.configureBindings()`?*
- *Why does the base container use `axis(2)` for rotation — what's on axis 2 in
  sim, and why does that break on a real Xbox pad?* (See
  [docs/controls.md](controls.md).)

---

## Lesson 6 — RobotContainer swap

### Goal
Point the entry point at your robot. This is the only edit to existing code.

> **Deep dive:** [docs/architecture.md](architecture.md) — how `Robot.java` and
> `RobotContainer.java` sit above the container subclass.

### Concept
`frc/robot/Robot.java` instantiates `frc.robot.RobotContainer`, a thin shell
that constructs *one* team container and delegates to it. Swapping the team
container in this one file switches robots without touching framework code.

### Work
Edit `src/main/java/frc/robot/RobotContainer.java`:
```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.comp.CompRobot;                 // ← changed import

public class RobotContainer {
  private final CompRobot robot;                 // ← changed type

  public RobotContainer() {
    robot = new CompRobot();                      // ← changed construction
  }

  public Command getAutonomousCommand() { return robot.getAutonomousCommand(); }
  public void resetToAllianceStart()    { robot.resetToAllianceStart(); }
}
```

### Checkpoint
- Project compiles end-to-end (`./gradlew compileJava`).
- No remaining references to `ExampleRobot` in this file.
- `ExampleRobot.java` / `ExampleRobotProfile.java` still exist, untouched —
  they're your reference, not dead code.

### Self-check question
- *If a teammate later wants to test a different swerve config without losing
  yours, what's the minimum change? Could two robots coexist in the codebase?*

---

## Lesson 7 — Sim verification

### Goal
Prove the whole stack works in the simulator before sending it to a real robot.

> **Deep dives:** [docs/testing.md](testing.md) — the test pyramid and what the
> smoke test checks · [docs/simulation.md](simulation.md) — Gradle flags
> (`-PwebUI`, `-PvisualTest`), Glass, AdvantageScope. To add your own coverage:
> **`/new-sim-test`**.

### Concepts
`./gradlew test` runs pyramid layers 1–3. The smoke test constructs your
`CompRobot` (because `RobotContainer` now does) and fails loudly if any constant
is missing, any CAN ID conflicts, or any constructor throws.

`./gradlew simulateJava` launches Glass + Driver Station + (with `-PwebUI`) the
web control panel. The robot starts **disabled** — click **Enable** in the
Driver Station panel (CLAUDE.md gotcha #7).

What to look for:
- **Field2d** shows a bumper-sized rectangle at `BLUE_START`. Wrong pose → wrong
  constants.
- **SmartDashboard → `CompElevator/mechanism`** shows the side view. Press A/B/Y
  and the carriage should track each preset smoothly. Overshoot/runaway →
  re-check the velocity sanity check (Lesson 3).
- **AdvantageScope → `Mechanisms3d/CompElevator`** shows the 3D view; both motor
  symbols (lead + follower mirror) should be visible.
- Driving with WASD or a stick — the robot translates without spinning. If it
  spins, module CAN IDs are crossed.

### Work
```bash
./gradlew test
./gradlew simulateJava            # add -PwebUI to drive from a browser tab
```

### Checkpoint
| Check | Pass criterion |
|---|---|
| Tests pass | `BUILD SUCCESSFUL`; the test that constructs `RobotContainer` → your `CompRobot` is green |
| Field2d at start pose | Robot rectangle visible at `BLUE_START` |
| Drive | WASD or stick moves the robot without unwanted yaw |
| Elevator A / B / Y | Carriage tracks to each preset smoothly |
| LEDs | Larson while the elevator moves, alliance solid when idle |

### Self-check / debugging questions
- *If the test fails with `NullPointerException` in `CompRobot`, what's the most
  common cause given the Lesson 5 lifecycle?*
- *If the elevator overshoots violently in sim, which `Settings` field is first
  to check?*
- *If the robot spins instead of going straight, which constants are suspect?*

---

## First hardware deploy — gotchas not caught in sim

These surface only on the real roboRIO. Add them to your pre-deploy checklist.

| Symptom | Root cause | Fix |
|---|---|---|
| Constant `PhotonVision coprocessor not found` errors filling the DS log | `createVision()` overridden but the coprocessor isn't connected | Remove the override until the coprocessor is installed and running; the base returns `null` (vision disabled). See [docs/vision.md](vision.md). |
| Translation works, rotation does nothing | Base `configureBindings()` maps rotation to `axis(2)` (left trigger on Xbox) | Override `drive.setDefaultCommand(...)` after `super`, using `controller.rightX()`. See [docs/controls.md](controls.md). |

---

## Lesson 8 — Hand-off

After the structure is proven in sim, these playbooks take you to a tuned,
calibrated, competition-ready robot:

- **`/calibrate-drive`** ([docs/calibration.md](calibration.md)) — SysId your
  real drive motors, update `TunerConstants` / `DriveConstants` with the gains.
- **`/tune-mechanism`** ([docs/mechanisms.md](mechanisms.md)) — characterize the
  elevator (`kG`, `kV`, `kA` via SysId; LQR `qelms`/`relms` over NetworkTables).
- **Autonomous** ([docs/auto.md](auto.md)) — BLine paths, the auto chooser,
  drive-to-pose buttons; patterns in `org/frc5010/examples/AutoRoutines.java`.
- **`/diagnose-log`** — pull a `.wpilog` after practice and read the anomaly
  flags (motor saturation, slip, vision dropouts).
- **Deploy** — `./gradlew deploy` to the roboRIO. Walk the hardware checklist in
  [docs/student-setup.md](student-setup.md) before driving for real.

---

## How to use this plan with Claude

- Work through one lesson at a time.
- At each **Checkpoint**, run the listed checks. If something fails, paste the
  error (or describe the behaviour) to Claude — it will revise that lesson
  before moving on.
- Answer the **Self-check questions** out loud before the next lesson; if you're
  stumped, ask Claude or open the linked deep dive.
- Don't skip ahead. Lesson 5 assumes Lesson 3's gotchas; the Lesson 7 sim
  verification won't make sense without Lesson 2's profile.
