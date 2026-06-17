# Build Your Robot — a guided lesson plan

> Part of the [**Lessons**](README.md) collection. This is the **intermediate** track: you'll
> turn this library into *your team's* real competition robot. Brand new to Java or coding? Do the
> beginner [Intake + Dual-Flywheel](intake-flywheel-lesson.md) lesson first — it builds a virtual
> robot from scratch and teaches the basics this lesson assumes.

Welcome! By the end of this lesson you'll have a complete, working robot configuration — a
**swerve drivetrain** plus a **two-motor elevator** with **status LEDs** — verified in the
simulator and ready to deploy to real hardware.

You'll build it **one small file at a time**, and **check that it works after every step** so you
never get far before catching a mistake.

---

## Before you start

**You'll get the most out of this if you have:**

- A working dev setup — WPILib installed locally ([Student Setup](../student-setup.md)) or a
  browser-only [Codespace](../codespaces.md). You should already be able to run
  `./gradlew simulateJava` on the unchanged project.
- A little comfort with the basics. If any of *class*, *package*, *command*, *subsystem*, or
  *units* are fuzzy, keep these open in a tab — they're short and we link them as we go:
  [Java basics](concepts/java-basics.md) · [Command-based programming](concepts/command-based.md)
  · [WPILib units](concepts/units.md) · [Control: PID & LQR](concepts/control-pid-lqr.md) ·
  [Git & GitHub](concepts/git-github.md) · [Simulation & the IO layer](concepts/simulation-and-io.md).
- **A robot to wire** (Kraken/TalonFX swerve) — or not! See the box below.

> **No robot yet? Follow along in simulation.** Everything in this lesson compiles, tests, and
> drives in the simulator without any hardware. Where a step asks for a real measurement or CAN
> ID, just use the placeholder values shown (or borrow them from the worked solution). The sim
> path never touches your real motor export, so you'll still see the full robot come to life
> on screen.

### How this lesson works

Three callouts appear throughout — the same ones used across the Lessons collection:

- **▶ Check it works** — stop and run a command. Each one tells you the exact command and **what
  you should see**. Don't move on until it passes.
- **💾 Save your progress** — a good moment to commit to Git (a "save point" you can return to).
  New to Git? → [Git & GitHub](concepts/git-github.md).
- **🩹 Temporary** — throwaway scaffolding a later step removes. Normal, real development.

> **Running commands — which `gradlew`?** Examples show `./gradlew …` (macOS / Linux /
> Codespaces). On **Windows local**, use `.\gradlew.bat …` from PowerShell. More:
> [environment](../environment.md).

> **Naming.** This plan uses the placeholder robot name **`Comp`** and the package
> **`frc.robot.comp`**. Replace both with your robot's real name everywhere — e.g.
> `frc.robot.viper` / `ViperRobot`.

> **Worked solution.** The [`tigershark`](https://github.com/clrozeboom/FRC5010Claude/tree/tigershark)
> branch is a complete, working example of this exact plan (Kraken swerve + two-motor elevator).
> Peek there whenever a step leaves you stuck.

---

## What you're building

A new FRC robot configuration:

- A **swerve drivetrain** (four Kraken/TalonFX modules, configured with Phoenix Tuner X).
- A **two-motor elevator** on one gearbox.
- **LEDs** that react to robot state.
- Vision left off until a camera is connected.

**You'll create these files** (all under `src/main/java/frc/robot/comp/`):
`TunerConstants.java`, `CompRobotProfile.java`, `CompElevator.java`, `CompLeds.java`,
`CompRobot.java`.

**You'll edit one existing file:** `src/main/java/frc/robot/RobotContainer.java` (three lines).

**You'll leave alone but copy from:** the examples in `org/frc5010/examples/`.

### The 8 lessons

| # | Lesson | Goal | Deep dives |
|---|---|---|---|
| 0 | [Branch setup](#lesson-0--branch-setup) | Make a safe place for your work | [Git & GitHub](concepts/git-github.md), [environment](../environment.md) |
| 1 | [TunerConstants](#lesson-1--tunerconstants) | Bring in the Phoenix Tuner X export | [configuration](../configuration.md) |
| 2 | [RobotProfile](#lesson-2--robotprofile) | Describe the drivetrain to the framework | [robot-profiles](../robot-profiles.md), [configuration](../configuration.md) · `/new-robot-profile` |
| 3 | [Elevator](#lesson-3--elevator) | Configure the two-motor elevator | [mechanisms](../mechanisms.md), [control: PID & LQR](concepts/control-pid-lqr.md) · `/new-mechanism` |
| 4 | [LEDs](#lesson-4--leds) | Map robot state to LED patterns | [leds](../leds.md) |
| 5 | [Robot container](#lesson-5--robot-container) | Wire subsystems + buttons | [controls](../controls.md), [command-based](concepts/command-based.md) |
| 6 | [RobotContainer swap](#lesson-6--robotcontainer-swap) | Point the program at your robot | [architecture](../architecture.md) |
| 7 | [Sim verification](#lesson-7--sim-verification) | Prove it works before hardware | [testing](../testing.md), [simulation](../simulation.md) · `/new-sim-test` |
| 8 | [Hand-off](#lesson-8--hand-off) | Tune, calibrate, deploy | [calibration](../calibration.md), [auto](../auto.md), [student-setup](../student-setup.md) |

> **▶ Check it works (before you change anything):** run `./gradlew simulateJava -PwebUI`, open
> the web UI (it prints a URL, usually `http://localhost:5800`), click **Enable**, and drive with
> the on-screen controls. This proves your tools work *before* you start. Stop the sim with
> `Ctrl-C` when done.

---

## Lesson 0 — Branch setup

### Goal
Make a Git branch so your robot's code evolves separately from the library.

### What to know first
The `main` branch is the **library**. Your robot is built on top of it. Working on a *branch*
lets you keep pulling library improvements without messy conflicts, and keeps team-specific
numbers out of `main`. New to branches? → [Git & GitHub](concepts/git-github.md).

### Do this
```bash
git switch -c comp        # name it after your robot
```

### ▶ Check it works
```bash
git status                # should say "On branch comp" and a clean tree
git log --oneline -1      # the newest commit (same as main's tip)
```

### 💾 Save your progress
Nothing to commit yet — the branch itself is your first save point. (Push it now if you like:
`git push -u origin comp`.)

---

## Lesson 1 — TunerConstants

### Goal
Bring the Phoenix Tuner X swerve export into the project as **generated code you don't hand-edit**.

### What to know first
Phoenix Tuner X's *Swerve Project Generator* produces **one file**, `TunerConstants.java`, holding
the four module definitions (`FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`) plus every
drive/steer/encoder CAN ID, the gyro ID, the CAN bus name, wheel radius, gear ratios, and
TunerX-calibrated gains.

**Why "don't hand-edit":** when you re-tune later (new encoder offsets after a crash, a new
gearbox), you re-run Tuner X and it **overwrites** this whole file — your hand edits would be
lost. So edits you want to keep go in the `RobotProfile` (Lesson 2), not here.

> **No hardware?** You can't generate a real export without a robot. To follow along in sim, grab
> the ready-made `TunerConstants.java` from the
> [`tigershark`](https://github.com/clrozeboom/FRC5010Claude/tree/tigershark) branch and use it
> as your placeholder — the simulator never reads it, it just needs to compile.

### Do this
1. Put the Tuner X file at `src/main/java/frc/robot/comp/TunerConstants.java`.
2. Make the first line `package frc.robot.comp;`.
3. **Read it once** and find these names — you'll use them next lesson: `kFrontLeftDriveMotorId`,
   `kFrontLeftSteerMotorId`, `kFrontLeftEncoderId`, the wheel-radius constant, `kDriveGearRatio`,
   `kSteerGearRatio`, `kCANBus`, and the four module objects.

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`. (If it fails, the most common cause is the package line not matching
the folder — it must read `package frc.robot.comp;`.)

### 💾 Save your progress
```bash
git add . && git commit -m "Add TunerConstants export"
```

### Think about it
- What CAN ID is your Front-Left **drive** motor on?
- Is `kCANBus` set to `""` (the RIO's CAN bus) or to a CANivore name?

---

## Lesson 2 — RobotProfile

### Goal
Describe your drivetrain to the framework: physical size, CAN IDs, gyro, starting position, and
the real-vs-simulation wiring.

> **Deep dives:** [robot-profiles](../robot-profiles.md) (the pattern and REAL/SIM branching) ·
> [configuration](../configuration.md) (every `SwerveConstants` field). The **`/new-robot-profile`**
> slash command is the interactive version of this lesson.

### What to know first
A `RobotProfile` holds your robot's **hardware facts** in one place. The framework reads it to:

1. **On a real robot** — pick the right motor/gyro classes and wire them.
2. **In simulation** — build a physics model of *your specific robot* (mass, bumper size, and
   wheel geometry change how it accelerates and bumps). Lie about the mass and the sim won't
   match reality.
3. **Always** — clamp top speed for driving and path planning.

**Why the factory "throws" on the real path** (a deliberate signpost, not a bug): the simulator
can fake motor specs, but a real robot needs *your* exact gear ratios, gains, and encoder offsets
— which the framework can't guess. So `SwerveFactory.build()` refuses the real path, and you wire
those four modules yourself in `createDrive()` (step 4 below). More:
[robot-profiles](../robot-profiles.md).

**Starting position:** you only write the **blue-alliance** start. The framework mirrors it for
red automatically.

### Do this
File: `src/main/java/frc/robot/comp/CompRobotProfile.java`

1. **Copy** `org/frc5010/examples/ExampleRobotProfile.java` to the new path. Change its package to
   `frc.robot.comp` and rename the class to `CompRobotProfile`.

2. **Fill in `CONSTANTS`** with your real numbers (every field explained in
   [configuration](../configuration.md)):

   ```java
   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
       .moduleType(ModuleType.TALON_FX)
       .gyroType(GyroType.PIGEON2)             // or NAVX
       .gyroCanId(/* TunerConstants.kPigeonId */)
       .trackWidth(Inches.of(/* MEASURED left-to-right wheel-center distance */))
       .wheelBase(Inches.of(/* MEASURED front-to-back wheel-center distance */))
       .wheelRadius(Inches.of(/* TunerX worn-wheel value */))
       .maxLinearSpeed(MetersPerSecond.of(/* measured top speed */))
       .maxAngularSpeed(RadiansPerSecond.of(/* measured top spin rate */))
       .robotMass(Pounds.of(/* weighed WITH bumpers + battery */))
       .bumperLength(Inches.of(/* outside-to-outside, front-to-back */))
       .bumperWidth(Inches.of(/* outside-to-outside, side-to-side */))
       .frontLeftIds(/* drive, steer, encoder — from TunerConstants */)
       .frontRightIds(/* … */)
       .backLeftIds(/* … */)
       .backRightIds(/* … */)
       .canBusName("")                         // match TunerConstants.kCANBus
       .odometryFrequency(Hertz.of(100))       // 250 for CANivore, 100 for the RIO bus
       .build();
   ```

   > **Why list CAN IDs here *and* in `TunerConstants`?** The framework uses these for simulation,
   > independently of the Tuner X file. If the two disagree, sim and real silently diverge — so
   > **make them match.**

3. **Set `BLUE_START`** to your team's real blue-alliance starting position (not the placeholder
   `(1.5, 2.0)`).

4. **Wire the real path.** Replace the `throw new UnsupportedOperationException(...)` in
   `createDrive()` with:

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
   return SwerveFactory.build(CONSTANTS, BLUE_START);   // simulation path
   ```

   Add imports for `GyroIO`, `GyroIOPigeon2`, `ModuleIO`, `ModuleIOTalonFXReal`, and
   `frc.robot.comp.TunerConstants` (your editor's quick-fix will offer them).

5. **Leave `createVision()` alone.** The base version returns `null`, which means "no vision" —
   exactly what you want until a camera is physically connected. (Overriding it without a camera
   spams errors every loop.) When you're ready: [vision](../vision.md) or `/new-vision-camera`.

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`. Then re-read your `frontLeftIds(...)` and confirm each number matches
the matching `TunerConstants.kFrontLeft*` value exactly.

### 💾 Save your progress
```bash
git add . && git commit -m "Add CompRobotProfile (drivetrain constants + real wiring)"
```

### Think about it
- Why does the **simulator** need `robotMass`? What feels wrong if you halve it?
- If you swapped the front-left and back-right `*Ids(...)`, what would the sim do when you push
  forward?

---

## Lesson 3 — Elevator

### Goal
Describe a two-motor elevator: its physics, geometry, and control style. You write **no control
math** — the library's `Elevator` class handles that. You just supply the numbers.

> **Deep dives:** [mechanisms](../mechanisms.md) (the full mechanism guide and every gotcha) ·
> [control: PID & LQR](concepts/control-pid-lqr.md) (what the two control styles mean). Slash
> commands: `/new-mechanism` to scaffold, `/tune-mechanism` to tune later.

### What to know first
**Two motors, one gearbox.** Both motors turn one drum. In `Settings` that's two fields: `canId`
(the lead motor) and `followerCanId` (the follower, which mirrors the lead).

**The #1 gotcha — `motorModel`.** The physics model defaults to *one* motor,
`DCMotor.getKrakenX60(1)`. With two motors you **must** set `DCMotor.getKrakenX60(2)` — that `2`
doubles the modeled torque. Forget it and the sim thinks your elevator is half as strong and your
tuning comes out wrong.

**`followerOpposed`** — set `true` when the follower is mounted facing the opposite way (common
with back-to-back gearboxes). It flips the follower's direction so both motors push the same way.
Get it wrong on a real robot and the motors fight each other and trip the breaker.

**Don't ask for impossible speed.** If `maxVelocity` is higher than the motors can actually reach,
the controller chases a target it can't hit and overshoots. Quick check (Kraken free speed ≈ 100
rotations/sec):
```
drumSpeed   (rot/s) = 100 / gearing
maxPossible (m/s)   = drumSpeed × drumCircumference(m)
```
Pick `maxVelocity` at roughly **80%** of `maxPossible`.

**Control style** (full comparison: [control: PID & LQR](concepts/control-pid-lqr.md)):
- `ControlStyle.LQR` (the default): the controller is built automatically from your physical
  numbers — no `kP`/`kI`/`kD` to guess. Best when you trust your mass and gearing. **Start here.**
- `ControlStyle.PROFILED_PID`: you hand-tune gains. Use it when the mass is uncertain.

**`kG` (gravity feedforward):** the voltage needed just to hold the carriage up against gravity. A
rough guess is fine for sim; you measure it for real later with `/tune-mechanism`.

### Do this
File: `src/main/java/frc/robot/comp/CompElevator.java`

1. **Copy** `org/frc5010/examples/mechanisms/ExampleElevator.java` to the new path; change the
   package and rename the class to `CompElevator`.

2. **Rewrite `settings()`** with your numbers:
   ```java
   var s = new Settings();
   s.name = "CompElevator";
   s.canId = /* lead Kraken CAN ID */;
   s.followerCanId = /* follower Kraken CAN ID */;
   s.followerOpposed = /* true if mounted facing opposite */;

   s.motorModel = DCMotor.getKrakenX60(2);            // ← CRITICAL: two motors
   s.gearReductionStages = new double[] { /* e.g. 5.0, 4.0  → 20:1 */ };
   s.drumCircumference = Inches.of(/* 2 × π × drum radius */);
   s.carriageMass = Kilograms.of(/* moving mass incl. end effector */);

   s.minHeight = Meters.of(0);
   s.maxHeight = Meters.of(/* travel */);
   s.startingHeight = Meters.of(/* height at power-on */);

   s.maxVelocity = MetersPerSecond.of(/* ≈ 80% of maxPossible, see check above */);
   s.maxAcceleration = MetersPerSecondPerSecond.of(/* start around 2.0 */);

   s.kG = Volts.of(/* rough guess; refine with sysId later */);
   return s;
   ```

3. **Write the speed check in a comment** above `s.maxVelocity` (your `drumSpeed`, `maxPossible`,
   and chosen `maxVelocity`) so a teammate can see your reasoning.

4. **Add an `isMoving()` helper** the LEDs will use:
   ```java
   private static final double MOVING_THRESHOLD_MPS = 0.02;   // 2 cm/s dead-band

   /** True when the carriage is actually moving (ignores tiny gear-lash wiggles). */
   public boolean isMoving() {
     return Math.abs(getVelocity().in(MetersPerSecond)) > MOVING_THRESHOLD_MPS;
   }
   ```
   Also add a public constant `SCORING_HEIGHT` — Lesson 5 binds a button to it.

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`. Then eyeball two things: `s.motorModel` says `getKrakenX60(2)` (not
`1`), and both `isMoving()` and `SCORING_HEIGHT` are `public`.

### 💾 Save your progress
```bash
git add . && git commit -m "Add CompElevator (two-motor, LQR)"
```

### Think about it
- If `followerOpposed` is wrong, what happens the moment you enable?
- Gearing 20:1, drum circumference 8.78 cm: what's the fastest the carriage can move, and a safe
  `maxVelocity`?

---

## Lesson 4 — LEDs

### Goal
Build LEDs that react to *your* robot's state, wired so the LED class doesn't depend on any
specific subsystem.

> **Deep dive:** [leds](../leds.md) — segments, animations (Larson, laser), and the state-mapping
> pattern.

### What to know first
The library's `LedStripSegments` owns the physical strip, splits it into named `Segment`s, and can
override the whole strip (for "disabled" animations). You write a short `periodic()` that turns
**robot state into LED patterns**.

The trick that keeps it clean: the LED class receives state through generic **suppliers**
(`BooleanSupplier`, `Supplier`) — little "ask me for the current value" functions — instead of
holding a reference to the elevator. That way the LEDs know about *light*, not about *elevators*.
New to suppliers/lambdas? → [Java basics §6](concepts/java-basics.md#6-lambdas-and-method-references).

### Do this
File: `src/main/java/frc/robot/comp/CompLeds.java`

1. **Copy** `org/frc5010/examples/DemoLeds.java`; change the package and rename to `CompLeds`.
2. **Remove everything about `DemoIntake`** — the `intakeExtended` parameter, `notifyShot()`, and
   the alliance-zone call.
3. **Accept state as constructor arguments**, e.g. `BooleanSupplier elevatorMoving`. Don't reach
   into the elevator from inside the LED class.
4. A good starter mapping:

   | Robot state | Pattern |
   |---|---|
   | Disabled, never enabled | Solid alliance colour (green if unknown) |
   | Disabled, was enabled before | Scrolling rainbow |
   | Enabled, elevator moving | Red Larson "scanner" |
   | Enabled, elevator idle | Solid alliance colour |

5. Keep the `periodic()` shape:
   ```java
   if (DriverStation.isEnabled()) everEnabled = true;
   if (DriverStation.isDisabled()) {
       setOverride(everEnabled ? RAINBOW : allianceSolid());
   } else {
       clearOverride();
       // your state → pattern mapping here
   }
   super.periodic();
   ```

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`, and a search of the file should find **no** mention of `DemoIntake`,
`Fuel`, or any subsystem name — only suppliers.

### 💾 Save your progress
```bash
git add . && git commit -m "Add CompLeds (state-driven, decoupled)"
```

### Think about it
- Why pass `BooleanSupplier elevatorMoving` instead of the whole elevator? What problem does the
  full reference invite?

---

## Lesson 5 — Robot container

### Goal
Wire the drive, the elevator, and the LEDs together, and bind controller buttons.

> **Deep dives:** [controls](../controls.md) — the input pipeline and the **`axis(2)` rotation
> gotcha** you must fix here · [command-based](concepts/command-based.md) — subsystems, commands,
> triggers.

### What to know first
Your robot is a subclass of `SwerveRobotContainer`. Two lifecycle facts matter:

1. The base constructor calls `configureBindings()` **before your subclass's constructor body
   runs**. So **create your subsystems *inside* `configureBindings()`** — a field you set after
   `super(...)` would still be `null` when bindings run.
2. `registerMechanism(thing::close)` registers cleanup so the automated tests can free CAN IDs and
   LED ports between runs. Skip it and the test suite gets flaky.

**Buttons:** `elevator.goToHeight(...)` returns a command that **holds** that height forever. Bind
one button per preset with `.onTrue(...)`. (Why "holds forever" is normal:
[command-based](concepts/command-based.md).)

**The `axis(2)` gotcha:** the base drive maps rotation to `axis(2)`, which on a real Xbox
controller is the **left trigger** — so rotation seems dead. You fix it by overriding the drive's
default command after `super.configureBindings()`, using `controller.rightX()` for rotation. The
exact code is in [controls](../controls.md).

### Do this
File: `src/main/java/frc/robot/comp/CompRobot.java`

1. **Copy** `org/frc5010/examples/ExampleRobot.java`; change the package and rename to `CompRobot`.
2. **Delete the demo stuff** — `configureDemoMechanisms()`, `demoElevator`, the `Example*`
   imports, `DemoIntake`, and the X-button binding. Keep the class shape (`extends
   SwerveRobotContainer`, `buildAutos()`, `configureBindings()`).
3. Point `super` at your profile:
   ```java
   public CompRobot() {
     super(SwerveRobotContainer.selectProfile("frc.robot.comp.CompRobotProfile"));
   }
   ```
4. Give it a placeholder auto (real autos later: [auto](../auto.md)):
   ```java
   @Override protected void buildAutos() { addAuto("None", Commands.none()); }
   ```
5. Write `configureBindings()` — drive fix + elevator + LEDs:
   ```java
   private static final int LED_PWM_PORT = 9;
   private CompElevator elevator;
   private CompLeds leds;

   @Override
   protected void configureBindings() {
     super.configureBindings();          // wires drive + standard buttons — don't skip!

     // Replace the default drive so rotation uses the right stick, not axis(2).
     // Full block: ../controls.md
     // drive.setDefaultCommand( ... controller.rightX() ... );

     elevator = new CompElevator();
     registerMechanism(elevator::close);
     controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
     controller.b().onTrue(elevator.goToHeight(CompElevator.SCORING_HEIGHT));
     controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));

     leds = new CompLeds(LED_PWM_PORT, elevator::isMoving);
     registerMechanism(leds::close);
   }
   ```

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`. Confirm: `super.configureBindings()` is the **first** line of the
method, the drive override uses `controller.rightX()` (not `axis(2)`), and both `elevator` and
`leds` are registered.

### 💾 Save your progress
```bash
git add . && git commit -m "Add CompRobot (subsystems + button bindings)"
```

### Think about it
- Why must subsystems be created *inside* `configureBindings()`?
- What breaks if you forget `super.configureBindings()`?

---

## Lesson 6 — RobotContainer swap

### Goal
Point the program at *your* robot. This is the only existing file you edit.

> **Deep dive:** [architecture](../architecture.md) — how `Robot.java` and `RobotContainer.java`
> sit above your container.

### What to know first
`frc/robot/Robot.java` creates a `frc.robot.RobotContainer`, a thin shell that builds **one** team
robot and forwards to it. Change that one class and the whole program runs your robot — no
framework files touched.

### Do this
Edit `src/main/java/frc/robot/RobotContainer.java`:
```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.comp.CompRobot;                 // ← changed

public class RobotContainer {
  private final CompRobot robot;                 // ← changed

  public RobotContainer() {
    robot = new CompRobot();                       // ← changed
  }

  public Command getAutonomousCommand() { return robot.getAutonomousCommand(); }
  public void resetToAllianceStart()    { robot.resetToAllianceStart(); }
}
```

### ▶ Check it works
```bash
./gradlew compileJava
```
Expect `BUILD SUCCESSFUL`, with no remaining mention of `ExampleRobot` in this file. (The
`Example*` files still exist — they're your reference, not dead code.)

### 💾 Save your progress
```bash
git add . && git commit -m "Point RobotContainer at CompRobot"
```

---

## Lesson 7 — Sim verification

### Goal
Prove the **whole robot** works in the simulator before it ever touches hardware.

> **Deep dives:** [testing](../testing.md) (what the automated tests check) ·
> [simulation](../simulation.md) (Glass, AdvantageScope, the `-PwebUI` / `-PvisualTest` flags) ·
> [simulation & the IO layer](concepts/simulation-and-io.md). Add your own test: `/new-sim-test`.

### What to know first
- `./gradlew test` runs the automated checks. Because `RobotContainer` now builds `CompRobot`, the
  smoke test constructs *your* robot and fails loudly if a constant is missing, a CAN ID
  conflicts, or a constructor throws.
- `./gradlew simulateJava` opens Glass + the Driver Station (add `-PwebUI` for the browser panel).
  The robot starts **disabled** — click **Enable** first (otherwise nothing moves; that's
  expected, not a bug).

### ▶ Check it works — run the tests
```bash
./gradlew test
```
Expect `BUILD SUCCESSFUL`. If a test throws `NullPointerException` inside `CompRobot`, you almost
certainly created a subsystem **outside** `configureBindings()` (Lesson 5).

### ▶ Check it works — drive the simulator
```bash
./gradlew simulateJava -PwebUI
```
Enable the robot, then walk this checklist:

| Look at | You should see | If not… |
|---|---|---|
| **Field2d** (the field view) | A bumper-sized rectangle sitting at your `BLUE_START` | Wrong pose → re-check `BLUE_START` / geometry in Lesson 2 |
| **Driving** (WASD / stick) | Robot translates without spinning on its own | Spins → module CAN IDs are crossed (Lesson 2) |
| **Elevator** (press A / B / Y) | Carriage smoothly tracks each preset height | Overshoot/runaway → re-check the speed sanity check (Lesson 3) |
| **LEDs** (web panel under the field) | Red Larson while the elevator moves, solid colour when idle | Wrong → re-check the mapping in Lesson 4 |

SmartDashboard → `CompElevator/mechanism` shows a side view of the elevator; AdvantageScope →
`Mechanisms3d/CompElevator` shows it in 3D (both motors visible). Stop the sim with `Ctrl-C`.

### 💾 Save your progress
```bash
git add . && git commit -m "Verify CompRobot in simulation"
git push -u origin comp
```

### Think about it
- If the robot spins instead of driving straight, which constants are suspect?
- If the elevator overshoots violently, which one `Settings` field do you check first?

---

## First hardware deploy — gotchas sim won't catch

These only show up on the real roboRIO. Keep them on a pre-deploy checklist:

| Symptom | Cause | Fix |
|---|---|---|
| Driver Station log floods with `PhotonVision coprocessor not found` | `createVision()` overridden but no camera connected | Remove the override until the camera is installed; the base disables vision. [vision](../vision.md) |
| Translation works but rotation does nothing | Rotation still mapped to `axis(2)` (Xbox left trigger) | Override the drive default after `super`, using `controller.rightX()`. [controls](../controls.md) |

---

## Lesson 8 — Hand-off

With the structure proven in sim, these playbooks take you to a tuned, competition-ready robot:

- **`/calibrate-drive`** ([calibration](../calibration.md)) — SysId your real drive motors and
  update the gains.
- **`/tune-mechanism`** ([mechanisms](../mechanisms.md)) — measure the elevator's `kG`/`kV`/`kA`
  and tune the controller.
- **Autonomous** ([auto](../auto.md)) — BLine paths, the auto chooser, drive-to-pose buttons.
- **`/diagnose-log`** — read a `.wpilog` after practice for motor saturation, slip, and dropouts.
- **Deploy** — `./gradlew deploy` to the roboRIO; walk the hardware checklist in
  [student-setup](../student-setup.md) before driving for real.

---

## Working with Claude as you go

- Do **one lesson at a time** and run its **▶ Check it works** before moving on.
- If a check fails, paste the error (or describe what you saw) to Claude — it'll fix that step
  before you continue.
- Stuck on a concept? Open the linked [concept page](README.md#core-concepts-beginner) or
  deep-dive doc, or compare against the `tigershark` worked-solution branch.
- Don't skip ahead — Lesson 7's sim check assumes everything from Lessons 2–5.

---

← Back to the [Lessons hub](README.md) · next: tune and deploy via [Lesson 8](#lesson-8--hand-off)
