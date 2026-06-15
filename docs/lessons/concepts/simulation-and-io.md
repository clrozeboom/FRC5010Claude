# Deep Dive: Simulation and the IO Layer

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). This explains
> how your robot can run with no hardware, and why the *same* code will later run on a real robot.

## 1. Why simulate

A simulator lets you write, run, and debug robot code without a physical robot — no field, no
batteries, no risk of breaking a real mechanism. You can test ideas in seconds, run automated
checks, and let many people work at once. This whole lesson runs in simulation.

---

## 2. The IO pattern: one behavior, three back-ends

Here's the key trick. A subsystem doesn't talk to a motor *directly*. It talks to a small
interface called an **IO**, and the library plugs in a different IO depending on where you're
running:

| Mode | IO it uses | What that does |
|---|---|---|
| **SIM** | a simulation IO | drives a physics model of the motor/mechanism |
| **REAL** | a hardware IO | drives the actual TalonFX motor |
| **REPLAY** | a no-op IO | feeds values back from a recorded log |

The selection happens once, from `RobotMode.get()`. Your `IntakeArm`, `Shooter`, etc. are written
**the same way for all three** — you never write "if simulation, else real." That's why the code
you build in this lesson is genuinely the code a real robot would run; only the IO underneath
changes.

```
IntakeArm  →  Arm (library)  →  MechanismIO  ──►  sim physics   (SIM)
                                              ├─►  real TalonFX  (REAL)
                                              └─►  log playback  (REPLAY)
```

---

## 3. Game-piece physics (IronMaple)

Driving and mechanisms are simulated, but so are the **game pieces**. The library uses a physics
engine (IronMaple) with a `SimulatedArena` that knows the field, its obstacles, and the Fuel
scattered on it. When your intake is deployed and you drive over Fuel, the simulation detects the
collision and "collects" it; when you fire, it launches a projectile that arcs and lands.

`FuelHandler` builds on the library's `SimRobotState`, which owns this intake simulation and even
publishes the held/scored counts for you. (Because game pieces are *simulation-only*, the lesson
creates `FuelHandler` inside a `RobotBase.isSimulation()` guard — on a real robot you'd create
your real mechanisms there instead.)

---

## 4. Seeing your robot: the web UI and AdvantageScope

Two ways to watch the simulation:

- **The web UI** (`-PwebUI`) — a browser panel on port **5800** with a field you can drive, an
  Enable button, the LED strip, and live state (`heldFuel`, `scoredFuel`, pose…). No install
  needed; great in Codespaces.
- **AdvantageScope** — a desktop app that draws a 3D view. Connect it to `localhost` and drag in
  signals like the arm's pose to watch the mechanism move in 3D.

```bash
./gradlew simulateJava -PwebUI       # drive from the browser
./gradlew simulateJava -PvisualTest  # auto-runs a scripted drive (no controller needed)
```

(On Windows, use `.\gradlew.bat ...`.)

---

## 5. Tests are another way to "run"

Automated tests run your robot in simulation *without* the GUI, asserting things in code. The
lesson's `LessonRobotSmokeTest` constructs the robot, deploys the intake, and scores — checking
that the arm rises and the held-Fuel count drops. Running `./gradlew test` is the fastest way to
confirm you didn't break anything.

> **A caution worth knowing:** the tests check *behavior*, but they don't render the browser
> graphics. After a change to LEDs or the web view, it's worth actually running
> `./gradlew simulateJava -PwebUI` and watching it — some things only show up when you drive the
> real flow.

---

## 6. Going to a real robot

When you're ready for hardware, you don't rewrite your subsystems. You supply a **robot profile**
with your real motor CAN IDs and physical measurements, set `RobotMode` to REAL, and the same
`IntakeArm`/`Shooter`/`FuelHandler` logic drives actual motors. The full walkthrough is in
[docs/student-setup.md](../../student-setup.md).

---

### Further reading (external)

- WPILib docs: [Introduction to Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
  · [Simulation GUI](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html)
- WPILib docs: [Addressable LEDs](https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html)
  — the `AddressableLED`/`LEDPattern` API your `StatusLeds` builds on.

---

### Back to the lesson

- [Module 1 — Java & command-based ideas](../intake-flywheel-lesson.md#module-1--java-and-command-based-ideas-youll-use)
- Related: [Command-based programming](command-based.md) · the project's [simulation guide](../../simulation.md)
