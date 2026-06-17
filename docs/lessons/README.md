# Lessons

This folder is the home for **guided, hands-on lessons**. Each lesson builds a working robot — or
a working robot feature — one small, runnable step at a time, and links out to the framework's
**deep-dive docs** for the concepts underneath. Start wherever fits your experience; the lessons
share a common set of [concept pages](#core-concepts-beginner) so ideas you learn in one carry
over to the next.

> New to the whole project? First get a sim running with
> [Student Setup](../student-setup.md) (local) or [Codespaces](../codespaces.md) (browser-only),
> then pick a lesson below.

---

## Choose a lesson

| Lesson | Level | You build | Runs on |
|---|---|---|---|
| [**Build Your Robot**](build-your-robot.md) | Intermediate | A real competition robot end-to-end — a Kraken/TalonFX swerve drivetrain plus a two-motor elevator and state-driven LEDs — through to hardware deploy, tuning, and calibration. | Sim → real hardware |
| [**Intake + Dual-Flywheel Robot**](intake-flywheel-lesson.md) | Beginner | A virtual robot built from scratch in `frc.robot`: an intake arm (LQR), two flywheels (onboard PID), status LEDs, and Fuel that launches along the robot's heading. Teaches Java **and** GitHub from zero. | Simulation only |

**Not sure which?** If you have a physical robot to wire up, take **Build Your Robot**. If you're
new to Java/coding and want to learn the framework safely with no hardware, take **Intake +
Dual-Flywheel** first — then Build Your Robot.

---

## Core concepts (beginner)

Short, from-scratch explainers the lessons link to whenever a basic idea first appears. Each ends
with external refreshers ([W3Schools Java](https://www.w3schools.com/java/),
[WPILib docs](https://docs.wpilib.org/)).

| Concept | Covers |
|---|---|
| [Java basics](concepts/java-basics.md) | classes, packages, `extends`/`super`, lambdas, `@Override` |
| [Git & GitHub](concepts/git-github.md) | branches, commits, push, pull requests, undo |
| [WPILib units](concepts/units.md) | `Degrees.of(...)`, `RPM.of(...)`, `.in(...)`, dimensions |
| [Command-based programming](concepts/command-based.md) | subsystems, commands, triggers, composition, requirements |
| [Control: feedforward, PID & LQR](concepts/control-pid-lqr.md) | closed-loop control and the two styles the lessons use |
| [Simulation & the IO layer](concepts/simulation-and-io.md) | how one codebase runs in SIM, REAL, and REPLAY |

---

## Framework reference deep dives

The lessons point here for the full detail behind each subsystem. These document the library, not
a step-by-step build:

| Area | Doc |
|---|---|
| `SwerveConstants` fields and units | [configuration](../configuration.md) |
| `RobotProfile` pattern, REAL/SIM wiring, vision | [robot-profiles](../robot-profiles.md) |
| Driver input pipeline, button bindings, the `axis(2)` gotcha | [controls](../controls.md) |
| TalonFX mechanisms (Arm/Flywheel/Elevator…), tuning, IO | [mechanisms](../mechanisms.md) |
| LED strips, segments, animations | [leds](../leds.md) |
| Simulation tools, Gradle flags, AdvantageScope | [simulation](../simulation.md) |
| Test pyramid, sim test patterns | [testing](../testing.md) |
| Path-following autos (BLine) | [auto](../auto.md) |
| Vision cameras (PhotonVision / Limelight) | [vision](../vision.md) |
| Motor calibration workflow | [calibration](../calibration.md) |
| IO-layer architecture overview | [architecture](../architecture.md) |

---

## How a lesson is structured

Every lesson here follows the same rhythm so they're easy to write and to follow:

1. **Goal** — what you'll have working at the end.
2. **Concepts first** — the ideas you need *before* writing code, each linked to a
   [concept page](#core-concepts-beginner) or [reference doc](#framework-reference-deep-dives).
3. **The work** — small steps, written so the project **compiles and runs after each one**.
4. **Checkpoint** — run the sim or the tests and confirm what you should see before moving on.

The beginner lessons also use three inline callouts (see the
[intake lesson](intake-flywheel-lesson.md) for examples):

- **▶ Try it now** — run it and see the step working.
- **🩹 Temporary code** — throwaway scaffolding a later step replaces (real development!).
- **💾 Checkpoint — commit** — a Git save point you can return to.

---

## Add your own lesson

This section is meant to be **built on**. To add a lesson:

1. Create `docs/lessons/<your-lesson>.md` and follow the
   [structure above](#how-a-lesson-is-structured).
2. Reuse the [concept pages](#core-concepts-beginner) instead of re-explaining basics — link to
   them at first mention. If you introduce a genuinely new basic concept, add a page under
   `docs/lessons/concepts/` and link it from [the list above](#core-concepts-beginner).
3. Put the lesson's **reference solution** code under `src/main/java/frc/robot/…` (the team's own
   package) and add a Layer-3 sim test under `src/test/java/frc/robot/…` so "check your work"
   stays honest. See [testing](../testing.md) and `/new-sim-test`.
4. Register the lesson in three indexes: the [Choose a lesson](#choose-a-lesson) table here, the
   "Deeper docs" table in [`CLAUDE.md`](../../CLAUDE.md), and the Documentation table in
   [`README.md`](../../README.md).

---

← Back to the [project README](../../README.md) · [docs index](../../CLAUDE.md#deeper-docs)
