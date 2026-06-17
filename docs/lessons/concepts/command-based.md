# Deep Dive: Command-Based Programming

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). This is the
> way WPILib robots organize behavior. Once it clicks, the lesson's `FuelHandler` reads like
> plain English.

## 1. The big picture

Every 20 milliseconds (50 times a second), WPILib runs a **scheduler**. The scheduler:

1. runs each **subsystem's** `periodic()` method,
2. checks every **trigger** (buttons, conditions) and schedules **commands** when they fire,
3. runs the currently scheduled commands.

You write three kinds of things and let the scheduler drive them:

- **Subsystems** — the parts of the robot.
- **Commands** — actions those parts perform.
- **Triggers** — conditions that start commands.

---

## 2. Subsystems

A subsystem owns a piece of hardware and represents it to the rest of the code. In the lesson,
`IntakeArm`, `IntakeRoller`, `Shooter`, `FuelHandler`, and `StatusLeds` are all subsystems (they
extend library classes that ultimately extend `SubsystemBase`).

Two facts matter:

- Each subsystem's `periodic()` runs **every cycle**, automatically. `StatusLeds.periodic()` uses
  this to repaint the strip from the latest device state.
- A subsystem can have **one default command** — what it does when nothing else has claimed it.
  The lesson sets the shooter's default to "spin to 0" so it idles down on its own after a shot.

---

## 3. Commands

A **command** is an action with a lifecycle the scheduler manages: it *initializes*, *executes*
each cycle, decides when it *isFinished*, and then *ends*. You rarely write those steps by hand —
the library mechanisms hand you ready-made commands:

```java
arm.goToAngle(IntakeArm.DEPLOY_ANGLE)   // a command: drive the arm to 110°
shooter.goToSpeed(Shooter.SHOOT_RPM)    // a command: spin the shooter to 3000 RPM
```

Some commands **finish** on their own (`runOnce` does its thing and ends). Others **run until
interrupted** — `goToAngle`/`goToSpeed` keep *holding* their target forever, which is exactly
what you want for "hold the arm out" or "keep the wheel spinning."

---

## 4. Requirements — the rule that makes coordination work

Every command **requires** the subsystems it controls. The scheduler enforces one rule:

> A subsystem can run **only one** command at a time.

So if a command that requires the arm is scheduled while another arm command is running, the new
one **interrupts** the old one. This is how the lesson's deploy/retract toggle works without any
`if` statements:

- `deployCommand()` requires the arm and roller and holds them *out / spinning*.
- `retractCommand()` requires the same arm and roller — scheduling it **interrupts** deploy and
  takes them *in / stopped*.

It's also why `scoreCommand()` is careful to require the *shooter* (and the fuel handler) but
**not** the arm/roller — so firing doesn't accidentally cancel your deploy.

---

## 5. Triggers — starting commands

A **trigger** is a condition. Bind a command to it:

```java
controller.leftBumper().onTrue(fuelHandler.deployCommand());   // press → start once
controller.a().onTrue(fuelHandler.scoreCommand());
roller.whileTrue(...);                                          // run while held, cancel on release
something.onFalse(...);                                         // run when it becomes false
```

- `onTrue` — schedule the command when the condition goes from false to true.
- `whileTrue` — run while true, cancel when it goes false.
- `onFalse` — schedule when it goes true→false.

`isAtSpeed(...)` on a flywheel returns a **Trigger** too — a condition you can wait on (next
section).

---

## 6. Composing commands

The real power: build big actions from small ones.

| Builder | Meaning |
|---|---|
| `Commands.parallel(a, b)` | run `a` and `b` at the same time; done when **both** finish |
| `Commands.sequence(a, b)` | run `a`, then `b` |
| `a.andThen(b)` | same as sequence: `a` then `b` |
| `a.beforeStarting(() -> ...)` | run a quick side-effect, then `a` |
| `Commands.waitUntil(cond)` | a command that just waits until `cond` is true |
| `a.raceWith(b)` | run both; finish as soon as **either** finishes (cancel the other) |
| `Commands.repeatingSequence(a, ...)` `.until(cond)` | loop until `cond` |

Here is the lesson's `deployCommand` read aloud: *"start collecting (a quick side-effect), then
in parallel swing the arm out and spin the roller."*

```java
public Command deployCommand() {
  return Commands.parallel(
          arm.goToAngle(IntakeArm.DEPLOY_ANGLE),
          roller.goToSpeed(IntakeRoller.INTAKE_RPM))
      .beforeStarting(() -> { intakeSimulation.startIntake(); intakeExtended = true; });
}
```

And `scoreCommand`: *"spin the shooter up; racing alongside that, wait until it's at speed, then
fire once."* When the fire finishes, the race ends, the shooter command is cancelled, and the
shooter's idle default spins it back down.

```java
public Command scoreCommand() {
  return shooter.goToSpeed(Shooter.SHOOT_RPM)
      .raceWith(Commands.waitUntil(shooter.isAtSpeed(Shooter.SHOOT_RPM, Shooter.RPM_TOLERANCE))
                        .andThen(fireCommand()));
}
```

Notice there are **no loops or flags** — composition expresses the logic directly. That's the
appeal of command-based code.

---

## 7. A subtlety: composed commands hold *all* their requirements

When you combine commands, the group reserves **every** subsystem any member needs, for the whole
time the group runs. That's why the lesson starts intake collection with a no-requirement
side-effect (`beforeStarting(() -> ...)`) instead of a fuel-handler command — so deploy only ties
up the arm and roller, leaving the fuel handler free for `fireCommand()` to use while deployed.

---

### Further reading (external)

The official WPILib command-based guide goes deeper on every idea here:

- [What Is Command-Based Programming?](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)
  ([section index](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html))
- [Subsystems](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html) ·
  [Commands](https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html)
- [Command Compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html)
  (`parallel`, `sequence`, `race`, `andThen`, …)
- [Binding Commands to Triggers](https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html)
  (`onTrue`, `whileTrue`, `onFalse`)

---

### Back to the lesson

- [Module 4 — FuelHandler: make the devices work together](../intake-flywheel-lesson.md#module-4--fuelhandler-make-the-devices-work-together)
- Related: [Control: PID vs LQR](control-pid-lqr.md) · [Java basics](java-basics.md)
