# Lesson: Build a Virtual Intake-Arm + Dual-Flywheel Robot

> Part of the [**Lessons**](README.md) collection. Next step after this one:
> [Build Your Robot](../build-your-robot.md) (wire a real competition robot).

Welcome! In this lesson you will build a **virtual robot** — one that runs entirely in
simulation, no hardware needed — that can:

- **Extend an intake on an arm** to pick up game pieces (called *Fuel* in the 2026 game),
- **Spin a roller** on that arm to pull Fuel in while the arm is deployed,
- **Spin up a second flywheel** and **launch Fuel in the direction the robot is facing** to score,
- **Show the status of each device on an LED strip.**

You will write this in Java, save your work with Git/GitHub, and run it in a simulator you can
drive from your web browser. **No prior Java or robot experience is assumed** — concepts are
explained as we go.

> This lesson *replaces* the library's built-in demo intake with your own version, which you'll
> build from scratch under `src/main/java/frc/robot/` (the package `frc.robot`). A complete
> **worked solution** lives on the
> [`claude/java-robot-intake-flywheel-lesson-2uyoev`](https://github.com/clrozeboom/FRC5010Claude/tree/claude/java-robot-intake-flywheel-lesson-2uyoev)
> branch — whenever this lesson says "the reference `X.java`", that's where to look. Peek if you
> get stuck, but try it yourself first.

---

## How this lesson works

You will build the robot **one small piece at a time**, and **run it after almost every step**
so you can see your progress. Three kinds of callouts appear throughout:

- **▶ Try it now** — stop and run the robot (or the tests) to see what you just did working.
- **🩹 Temporary code** — throwaway code we add just to test the current piece. It gets replaced
  by a later step. Writing temporary scaffolding and deleting it later is *normal, real
  development* — don't be afraid of it.
- **💾 Checkpoint — commit** — a good moment to save your work to Git. Each commit is a "save
  point" you can return to if a later step breaks something.

Two commands you'll use constantly:

| Command | What it does |
|---|---|
| `./gradlew simulateJava -PwebUI` | Launches the robot in simulation with a browser control panel on port **5800**. (On Windows, use `.\gradlew.bat simulateJava -PwebUI`.) |
| `./gradlew test` | Compiles your code and runs the automated tests — a fast way to check you didn't break anything. |

### 📚 Concept deep dives

This lesson stays hands-on and keeps explanations short. Whenever a basic idea first comes up,
you'll see a **📚 Deep dive** link to a companion page that explains it slowly and from scratch —
and each of those pages ends with **external refreshers** (free
[W3Schools Java](https://www.w3schools.com/java/) tutorials and official
[WPILib docs](https://docs.wpilib.org/)). Read them when you want more, skip them when you're
cruising. The full set:

| Deep dive | When it helps |
|---|---|
| [Java basics](concepts/java-basics.md) | classes, packages, `extends`, lambdas — the Java this lesson uses |
| [Git & GitHub](concepts/git-github.md) | branches, commits, push, pull requests |
| [WPILib units](concepts/units.md) | `Degrees.of(110)`, `RPM.of(3000)`, and why typed units exist |
| [Command-based programming](concepts/command-based.md) | subsystems, commands, triggers, composing them |
| [Control: feedforward, PID & LQR](concepts/control-pid-lqr.md) | how the arm (LQR) and flywheels (PID) actually steer themselves |
| [Simulation & the IO layer](concepts/simulation-and-io.md) | how the same code runs in sim and on real hardware |

---

## Module 0 — Get set up (and meet Git/GitHub)

### Your tools

If you have never set up this project, follow **[docs/student-setup.md](../student-setup.md)**
(local install) or **[docs/codespaces.md](../codespaces.md)** (run in a browser, nothing to
install). Come back here once `./gradlew simulateJava -PwebUI` opens a field you can drive.

### Git and GitHub in 90 seconds

> 📚 **Deep dive:** [Git & GitHub](concepts/git-github.md) — staging vs. committing, branches,
> remotes, pull requests, and how to undo safely. Hands-on refresher:
> [W3Schools Git tutorial](https://www.w3schools.com/git/).

**Git** is a tool that takes snapshots of your code so you can save progress, undo mistakes, and
share work. **GitHub** is a website that stores those snapshots online.

- A **repository** ("repo") is your project folder, tracked by Git.
- A **fork** is your own copy of someone else's repo on GitHub.
- A **clone** is a copy of a repo on your computer.
- A **commit** is one saved snapshot, with a short message describing what changed.
- A **branch** is a separate line of work, so your experiments don't disturb the main code.
- A **pull request** ("PR") asks to merge your branch's changes into the main code.

Make a branch for this lesson so all your work is together:

```bash
git switch -c my-intake-lesson
```

The basic save loop, which you'll repeat at every 💾 checkpoint:

```bash
git add .                                   # stage your changes
git commit -m "Add intake arm subsystem"    # save a snapshot with a message
git push -u origin my-intake-lesson          # upload it to GitHub
```

**▶ Try it now:** run `./gradlew simulateJava -PwebUI` on the **unchanged** project. Open the web
UI (it prints a URL, usually `http://localhost:5800`), click **Enable**, and drive with the
on-screen controls. This proves your tools work *before* you change anything.

**💾 Checkpoint — commit** your fresh branch (even with no changes yet) so it exists on GitHub.

---

## Module 1 — Java and "command-based" ideas you'll use

You don't need to know all of Java. Here are just the ideas this lesson uses. Each links to a
deep dive if you want the slow version.

- **Class** — a blueprint. `public class Shooter { ... }` describes what a Shooter *is* and can
  *do*. An **object** is one real Shooter built from that blueprint with `new Shooter()`.
- **Package** — a folder/namespace for classes. *Your* robot code lives in `frc.robot`. The
  reusable **library** lives in `org.frc5010.common`. The library's **examples/demos** live in
  `org.frc5010.examples` — we will *not* depend on those; you're building your own.
- **`extends`** — "is a kind of." `class Shooter extends Flywheel` means your Shooter gets all of
  the library `Flywheel`'s abilities for free, and you only fill in your robot's numbers.
  &nbsp;📚 *Deep dive:* [Java basics](concepts/java-basics.md) (classes, packages, `extends`, `super`, `@Override`);
  refresher: W3Schools [Classes](https://www.w3schools.com/java/java_classes.asp) &
  [Inheritance](https://www.w3schools.com/java/java_inheritance.asp).
- **Lambdas / method references** — tiny inline functions. `drive::getPose` means "a function
  that calls `drive.getPose()`." We pass these around so one part of the robot can *ask* another
  for live information (like the robot's position) whenever it needs it.
  &nbsp;📚 *Deep dive:* [Java basics §6](concepts/java-basics.md#6-lambdas-and-method-references).
- **Units** — instead of bare numbers, WPILib uses typed units so you can't mix up degrees and
  rotations: `Degrees.of(110)`, `RPM.of(3000)`, `Meters.of(0.5)`.
  &nbsp;📚 *Deep dive:* [WPILib units](concepts/units.md); reference:
  [WPILib Java Units Library](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).
- **Subsystem, Command, Trigger** — the heart of robot code:
  - A **Subsystem** is a part of the robot (the arm, a flywheel, the LEDs). Each runs a
    `periodic()` method every 20 ms.
  - A **Command** is an action a subsystem can perform ("spin up to 3000 RPM", "go to 110°").
    Commands can be combined: `parallel(a, b)` runs both at once; `sequence(a, b)` runs `a` then
    `b`; `waitUntil(condition)` pauses until something becomes true.
  - A **Trigger** is a condition (a button press, "is the wheel at speed?") that *schedules* a
    command when it happens.
  - &nbsp;📚 *Deep dive:* [Command-based programming](concepts/command-based.md) (the scheduler,
    requirements, and how to compose commands); reference:
    [WPILib Command-Based](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html).
- **Simulation vs. real ("IO")** — the same code runs in the simulator and on a real robot; a
  thin layer underneath swaps simulated motors for real ones. You write the behavior once.
  &nbsp;📚 *Deep dive:* [Simulation & the IO layer](concepts/simulation-and-io.md).

That's enough to start. You'll see each idea in real code below.

---

## Module 2 — Your robot shell, and your first moving part (the arm)

### Step 2a — Create your robot

Your robot is a class that `extends` the library's `SwerveRobotContainer`, which already gives
you a driveable swerve base, keyboard/controller input, and an autonomous chooser.

Create `src/main/java/frc/robot/LessonRobot.java`:

```java
package frc.robot;

import org.frc5010.common.profiles.SwerveRobotContainer;

public class LessonRobot extends SwerveRobotContainer {
  public LessonRobot() {
    // Reuse the placeholder team hardware profile for the drivetrain numbers.
    super(SwerveRobotContainer.selectProfile("org.frc5010.examples.ExampleRobotProfile"));
  }
}
```

Now tell the program to run *your* robot. Open `src/main/java/frc/robot/RobotContainer.java` and
change the one line that builds the robot so it builds a `LessonRobot` instead of the demo
`ExampleRobot`:

```java
private final LessonRobot robot;

public RobotContainer() {
  robot = new LessonRobot();
}
```

(Update the `import` and field type to match; your editor will offer a quick-fix.)

**▶ Try it now:** `./gradlew simulateJava -PwebUI`, Enable, and drive. It behaves exactly like
before — that's the point. You now have a working robot *shell* that's yours to build on.

**💾 Checkpoint — commit** ("Add LessonRobot shell").

### Step 2b — Add the arm (and meet LQR)

The arm is a real motor mechanism. The library's `Arm` class does all the control math; you just
supply this robot's physical numbers by `extends`-ing it.

Create `src/main/java/frc/robot/subsystems/IntakeArm.java`:

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import org.frc5010.common.mechanisms.Arm;

public class IntakeArm extends Arm {
  public static final int CAN_ID = 41;
  public static final Angle DEPLOY_ANGLE = Degrees.of(110);  // out, ready to collect
  public static final Angle RETRACT_ANGLE = Degrees.of(0);   // stowed
  public static final Angle ANGLE_TOLERANCE = Degrees.of(5);

  public IntakeArm() { super(settings()); }

  private static Settings settings() {
    var s = new Settings();
    s.name = "IntakeArm";
    s.canId = CAN_ID;
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {10, 5}; // 50:1
    s.length = Meters.of(0.5);
    s.mass = Kilograms.of(3.0);
    s.minAngle = Degrees.of(-10);
    s.maxAngle = Degrees.of(120);
    s.startingAngle = RETRACT_ANGLE;
    s.maxVelocity = DegreesPerSecond.of(220);
    s.maxAcceleration = DegreesPerSecondPerSecond.of(440);
    s.kG = Volts.of(0.35);  // volts needed to hold the arm level (gravity)
    return s;
  }
}
```

**Concept — LQR control.** We did *not* set a control style, so the arm uses the library default,
**LQR**. LQR builds an optimal controller automatically from the physical numbers you gave it
(mass, length, gearing, motor). The payoff: **there are no PID gains to hand-tune.** Get the
physics roughly right and it just works. (In Module 3 you'll meet the *other* style, PID, on the
flywheels — so you'll have used both.)

> 📚 **Deep dives:** [Control: feedforward, PID & LQR](concepts/control-pid-lqr.md) explains what
> LQR is doing and why `kG` is still needed; [Java basics](concepts/java-basics.md#5-inheritance-extends-and-super)
> covers the `extends`/`super` pattern this class uses; [WPILib units](concepts/units.md) covers
> `Degrees.of(...)` and `Volts.of(...)`. Reference:
> [WPILib State-Space Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html)
> ("From PID to Model-Based Control").

To see the arm move, bind a button to it — **temporarily**. In `LessonRobot`, override
`configureBindings()`:

```java
@Override
protected void configureBindings() {
  super.configureBindings(); // keep keyboard/controller drive
  if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;

  var arm = new frc.robot.subsystems.IntakeArm();   // 🩹 temporary
  controller.leftBumper().onTrue(arm.goToAngle(IntakeArm.DEPLOY_ANGLE));   // 🩹 temporary
  controller.rightBumper().onTrue(arm.goToAngle(IntakeArm.RETRACT_ANGLE)); // 🩹 temporary
}
```

> **🩹 Temporary code:** this `arm` and these two bindings are scaffolding so you can watch the
> arm move *right now*. In Module 4 the arm moves *inside* the `FuelHandler` and these lines go
> away.

**▶ Try it now:** launch the sim, connect **AdvantageScope** (see student-setup), and watch the
3D arm swing when you press the left/right bumper buttons in the web UI. You just drove a real
control loop!

**💾 Checkpoint — commit** ("Add IntakeArm, LQR, temporary bindings").

---

## Module 3 — Add the two flywheels (and meet PID)

A **flywheel** is a spinning wheel. You have two:

- the **intake roller** (pulls Fuel in), and
- the **shooter** (launches Fuel out).

Both use the library `Flywheel` class. This time we choose **`ControlStyle.PROFILED_PID`** —
*onboard* PID, the classic approach — so you learn it alongside the arm's LQR.

Create `src/main/java/frc/robot/subsystems/IntakeRoller.java`:

```java
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5010.common.mechanisms.ControlStyle;
import org.frc5010.common.mechanisms.Flywheel;

public class IntakeRoller extends Flywheel {
  public static final int CAN_ID = 42;
  public static final AngularVelocity INTAKE_RPM = RPM.of(2000);

  public IntakeRoller() { super(settings()); }

  private static Settings settings() {
    var s = new Settings();
    s.name = "IntakeRoller";
    s.canId = CAN_ID;
    s.controlStyle = ControlStyle.PROFILED_PID; // onboard PID, not LQR
    s.motorModel = DCMotor.getKrakenX60(1);
    s.gearReductionStages = new double[] {1.0}; // direct drive
    s.diameter = Inches.of(2);
    s.mass = Kilograms.of(0.4);
    s.kP = 0.15;  // proportional gain
    s.kV = 0.115; // velocity feedforward
    return s;
  }
}
```

**Concept — PID (with feedforward) for speed.** PID nudges the motor based on the *error* between
the speed you want and the speed you have (`kP`), while a **feedforward** `kV` predicts most of
the voltage needed up front. For a flywheel the feedforward does nearly all the work; a good
starting guess is `kV ≈ 12 volts ÷ free speed in rotations/second`, and `kP` just cleans up the
rest. Unlike LQR, *you* pick these numbers — but they're easy to reason about.

> 📚 **Deep dive:** [Control: feedforward, PID & LQR](concepts/control-pid-lqr.md#3-pid) walks
> through P, I, D and feedforward one term at a time, and compares PID with the arm's LQR.
> Reference: WPILib [Introduction to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
> and [DC Motor Feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html).

Now create `Shooter.java` the same way (CAN 43, 4-inch wheel, also `PROFILED_PID`, with a
`SHOOT_RPM = RPM.of(3000)` constant and a `RPM_TOLERANCE = RPM.of(150)`).

Wire each to a button **temporarily** so you can watch them spin:

```java
var roller = new frc.robot.subsystems.IntakeRoller();   // 🩹 temporary
var shooter = new frc.robot.subsystems.Shooter();       // 🩹 temporary
controller.x().whileTrue(roller.goToSpeed(IntakeRoller.INTAKE_RPM)); // 🩹 temporary
controller.y().whileTrue(shooter.goToSpeed(Shooter.SHOOT_RPM));      // 🩹 temporary
```

**▶ Try it now:** in the sim, hold X/Y and watch each wheel's speed climb (look at the
`IntakeRoller/GoalRPM` and `Shooter/GoalRPM` values in the web UI / AdvantageScope). You've now
used **both** control styles — LQR on the arm, PID on the wheels.

**💾 Checkpoint — commit** ("Add roller + shooter, PROFILED_PID, temporary bindings").

---

## Module 4 — `FuelHandler`: make the devices work together

Right now each device is wired to its own button. Real robots coordinate them: deploying the
intake should swing the arm out *and* spin the roller *and* start collecting. Scoring should spin
the shooter up, wait until it's fast enough, *then* launch. We'll put that coordination in one
subsystem, `FuelHandler`.

`FuelHandler` `extends` the library's `SimRobotState`, which **for free** gives you:

- the IronMaple **intake simulation** that models Fuel collection,
- automatic publishing of *held / extended / scored* counts to the web UI.

You add the three devices and the commands that combine them. Create
`src/main/java/frc/robot/subsystems/FuelHandler.java`. The skeleton:

```java
public class FuelHandler extends SimRobotState implements AutoCloseable {
  private final IntakeArm arm = new IntakeArm();
  private final IntakeRoller roller = new IntakeRoller();
  private final Shooter shooter = new Shooter();

  public FuelHandler(AbstractDriveTrainSimulation driveSim,
                     Supplier<Pose2d> poseSupplier, Field2d field2d) {
    super(IntakeSimulation.OverTheBumperIntake("Fuel", driveSim,
            Inches.of(24), Inches.of(12), IntakeSide.FRONT, 50),
          poseSupplier, /* intake overlay offset */ 0.9,
          field2d != null ? field2d.getObject("Intake") : null);
    intakeSimulation.setGamePiecesCount(8); // start preloaded

    // Idle defaults so nothing sags or free-spins at startup.
    arm.setDefaultCommand(arm.goToAngle(IntakeArm.RETRACT_ANGLE));
    roller.setDefaultCommand(roller.goToSpeed(RPM.of(0)));
    shooter.setDefaultCommand(shooter.goToSpeed(RPM.of(0)));
  }
  ...
}
```

**Concept — composing commands.** Build *deploy* by running the arm and roller at the same time,
and starting collection just before. (📚 Deep dive:
[Command-based programming](concepts/command-based.md) — the scheduler, requirements, and every
builder used here; reference:
[WPILib Command Compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).)

```java
public Command deployCommand() {
  return Commands.parallel(
          arm.goToAngle(IntakeArm.DEPLOY_ANGLE),
          roller.goToSpeed(IntakeRoller.INTAKE_RPM))
      .beforeStarting(() -> { intakeSimulation.startIntake(); intakeExtended = true; });
}
```

`retractCommand()` is the mirror image (arm to `RETRACT_ANGLE`, roller to 0, `stopIntake()`).

**Concept — the "at speed" gate.** Scoring should wait until the shooter is actually fast enough.
`Flywheel` gives you `isAtSpeed(...)`, a Trigger you can wait on (📚 deep dive:
[tolerances & `isAtSpeed`](concepts/control-pid-lqr.md#6-at-speed--at-angle-tolerances)):

```java
public Command scoreCommand() {
  return shooter.goToSpeed(Shooter.SHOOT_RPM)
      .raceWith(Commands.waitUntil(shooter.isAtSpeed(Shooter.SHOOT_RPM, Shooter.RPM_TOLERANCE))
                        .andThen(fireCommand()));
}
```

This spins the shooter, and *in parallel* waits until it reaches speed and then fires one piece;
when the fire finishes, the race ends and the shooter spins back down (its idle default resumes).

**Concept — launching along the robot heading.** `fireCommand()` calls a small helper that takes
one Fuel out of the intake and creates a projectile aimed **in the direction the robot is
facing**:

```java
double heading = pose.getRotation().getRadians();         // which way the robot points
Translation2d launchPos = new Translation2d(
    pose.getX() + BUMPER_HALF_M * Math.cos(heading),
    pose.getY() + BUMPER_HALF_M * Math.sin(heading));
RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(launchPos, new Translation2d(),
    new ChassisSpeeds(), new Rotation2d(heading) /* aim = heading */,
    Meters.of(LAUNCH_HEIGHT_M), MetersPerSecond.of(speed), Degrees.of(SHOT_ELEVATION_DEG));
```

When you're inside your scoring zone and aimed at the hub, the shot scores (it bumps a counter
the web UI reads). The full helper — including the launch-speed math and the scoring callback —
is in the reference `FuelHandler.java`; read its comments.

Finally, **replace the temporary bindings** from Modules 2–3 with the real ones. In `LessonRobot`,
create the `FuelHandler` (it needs the physics sim, the robot pose, and the field for drawing
Fuel) and bind the three actions:

```java
drive.getDriveTrainSimulation().ifPresent(driveSim -> {
  fuelHandler = new FuelHandler(driveSim, drive::getPose, drive.getField2d());
  registerMechanism(fuelHandler::close); // free motors at shutdown / in tests

  controller.leftBumper().onTrue(fuelHandler.deployCommand());
  controller.rightBumper().onTrue(fuelHandler.retractCommand());
  controller.a().onTrue(fuelHandler.scoreCommand());
});
```

> Delete the 🩹 temporary `arm`/`roller`/`shooter` locals and their X/Y bindings — `FuelHandler`
> owns the devices now. This is the "backtrack" the lesson promised: scaffolding served its
> purpose and goes away.

**▶ Try it now:** in the sim, **Enable**, press **left bumper** to deploy, drive over Fuel and
watch `heldFuel` climb on the web UI's state panel. Aim at the hub, press **A**, and watch
`heldFuel` drop and `scoredFuel` rise as a Fuel arcs out along your heading.

**💾 Checkpoint — commit** ("Add FuelHandler; wire deploy/retract/score").

---

## Module 5 — Status LEDs, one device at a time

Lastly, show each device's status on a 30-LED strip split into three 10-LED segments. `StatusLeds`
`extends` the library `LedStripSegments`, which manages the strip; you choose a *pattern* for each
segment every cycle based on the device state.

> 📚 The strip is built on WPILib's LED API — reference:
> [WPILib Addressable LEDs](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html).
> The per-cycle `periodic()` idea is in [Command-based programming](concepts/command-based.md#2-subsystems).

Create `src/main/java/frc/robot/subsystems/StatusLeds.java`. Build it up one segment at a time —
wire the **arm** segment first and run it, then add the **roller** segment, then the **shooter**
segment and the disabled override. The core idea:

```java
@Override
public void periodic() {
  if (DriverStation.isDisabled()) {
    setOverride(everEnabled ? RAINBOW : GREEN);   // whole-strip status while disabled
  } else {
    clearOverride();
    armSeg.setPattern(deployed.getAsBoolean() ? ARM_DEPLOYED : GREEN);
    rollerSeg.setPattern(rollerSpinning.getAsBoolean() ? ORANGE : OFF);
    shooterSeg.setPattern(shooterPattern());      // yellow → green → flash on a shot
  }
  super.periodic();  // render the chosen patterns
}
```

The device states come in as `BooleanSupplier` lambdas — the LEDs *ask* the `FuelHandler` for its
status each cycle. `ARM_DEPLOYED` uses the library's `LedAnimations.larson(...)` (a red "scanner"
sweep); the shot flash uses `LedAnimations.laser(...)`. Add small status accessors to
`FuelHandler` (`isIntakeExtended()`, `isRollerSpinning()`, `isShooterSpinning()`,
`isShooterAtSpeed()`) and construct the LEDs in `LessonRobot`:

```java
statusLeds = new StatusLeds(9 /* PWM port */,
    fuelHandler::isIntakeExtended, fuelHandler::isRollerSpinning,
    fuelHandler::isShooterSpinning, fuelHandler::isShooterAtSpeed);
registerMechanism(statusLeds::close);
// Flash the strip when you actually fire:
controller.a().onTrue(Commands.runOnce(() -> {
  if (fuelHandler.getHeldFuel() > 0) statusLeds.notifyShot();
}).andThen(fuelHandler.scoreCommand()));
```

**▶ Try it now:** the web UI draws the LED strip under the field. Deploy → the arm segment turns
to a red scanner and the roller segment turns orange. Press A → the shooter segment goes
yellow→green as it spins up, then flashes white when it fires. Disable → the whole strip shows the
rainbow.

**💾 Checkpoint — commit** ("Add StatusLeds, one segment per device").

---

## Module 6 — Polish, test, and ship

1. **Clean up.** Make sure all 🩹 temporary code is gone and only the real `FuelHandler` /
   `StatusLeds` wiring remains.
2. **Add a tiny auto** (optional). In `LessonRobot.buildAutos()`, register a "Score Preload"
   routine that fires until empty — see the reference for the one-liner.
3. **Run the tests:** `./gradlew test`. The worked-solution branch includes a sample
   `LessonRobotSmokeTest` showing how to verify, in code, that deploy raises the arm and scoring
   launches a Fuel — write one like it. Green means your robot constructs, deploys, and scores
   correctly.
4. **Full run-through:** `./gradlew simulateJava -PwebUI` and play the whole flow — drive, deploy,
   collect, aim, score — watching the LEDs and the `heldFuel` / `scoredFuel` numbers. (📚 How the
   simulator works: [Simulation & the IO layer](concepts/simulation-and-io.md); reference:
   [WPILib Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html).)
5. **Ship it.** Commit everything, push, and open a pull request (📚 deep dive:
   [Git & GitHub §7](concepts/git-github.md#7-pull-requests)):
   ```bash
   git add . && git commit -m "Finish intake + dual-flywheel lesson robot"
   git push -u origin my-intake-lesson
   ```
   Then on GitHub, click **Compare & pull request** to open your PR.

---

## Check your work

The files you'll have built (and the complete, working **worked solution** to compare against on
the [`claude/java-robot-intake-flywheel-lesson-2uyoev`](https://github.com/clrozeboom/FRC5010Claude/tree/claude/java-robot-intake-flywheel-lesson-2uyoev)
branch):

| File | What it is |
|---|---|
| `src/main/java/frc/robot/subsystems/IntakeArm.java` | Arm (LQR) |
| `src/main/java/frc/robot/subsystems/IntakeRoller.java` | Intake roller flywheel (onboard PID) |
| `src/main/java/frc/robot/subsystems/Shooter.java` | Shooter flywheel (onboard PID) |
| `src/main/java/frc/robot/subsystems/FuelHandler.java` | Ties the devices together; deploy / retract / score; launch-along-heading |
| `src/main/java/frc/robot/subsystems/StatusLeds.java` | One LED segment per device |
| `src/main/java/frc/robot/LessonRobot.java` | Your robot container and button bindings |
| `src/test/java/frc/robot/LessonRobotSmokeTest.java` | Automated checks |

## What you learned

- Java basics: classes, packages, `extends`, lambdas, typed units.
- Git/GitHub: branch, commit, push, pull request.
- Command-based robot code: subsystems, commands, triggers, and composing them with
  `parallel` / `sequence` / `waitUntil`.
- **Two control styles:** LQR (the arm — automatic, model-based) and PID with feedforward
  (the flywheels — onboard, hand-set gains).
- Game-piece simulation: collecting and launching, and launching **in the direction the robot
  is facing**.
- Status LEDs driven by live device state.

### Go deeper on the basics

If any idea here felt fast, the companion deep dives explain it from scratch (each ends with
external W3Schools / WPILib refreshers):
[Java basics](concepts/java-basics.md) ·
[Git & GitHub](concepts/git-github.md) ·
[WPILib units](concepts/units.md) ·
[Command-based programming](concepts/command-based.md) ·
[Control: feedforward, PID & LQR](concepts/control-pid-lqr.md) ·
[Simulation & the IO layer](concepts/simulation-and-io.md)

External references to bookmark: the free [W3Schools Java tutorial](https://www.w3schools.com/java/)
and the official [WPILib documentation](https://docs.wpilib.org/).

### Where to go next

- [docs/mechanisms.md](../mechanisms.md) — go deeper on Arm/Flywheel, tuning, and the IO layer.
- [docs/leds.md](../leds.md) — more LED patterns and segment tricks.
- [docs/auto.md](../auto.md) — build real driving autos with BLine paths.
- [docs/student-setup.md](../student-setup.md) — wire this code to a real robot.
