# Deep Dive: Control — Feedforward, PID, and LQR

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). The lesson
> uses **two** ways to control a motor: LQR (on the arm) and PID (on the flywheels). This page
> explains what they are and why you'd pick each.

## 1. Open loop vs. closed loop

**Open loop** means you just set a voltage and hope: "apply 6 volts." If the load changes — the
arm is heavier than expected, the wheel is dragging — you get the wrong result and never correct.

**Closed loop** means you *measure* what actually happened and *correct*. You tell the controller
a **setpoint** ("go to 110°", "spin at 3000 RPM"); it reads the **measurement** (where the arm
actually is, how fast the wheel actually spins); and it computes the **error** = setpoint −
measurement, then drives the motor to shrink that error. Both PID and LQR are closed-loop.

---

## 2. Feedforward: predict first, correct second

A good controller doesn't wait for error to build up — it **predicts** most of the voltage needed
and applies it immediately. That prediction is **feedforward**. Common terms:

- **kV** — volts per unit of *speed*. To spin a wheel at a given RPM you need roughly a fixed
  voltage; kV captures that. A solid first guess: `kV ≈ 12 volts ÷ free speed in rotations/second`.
- **kG** — volts to fight *gravity*. An arm sticking out horizontally needs constant voltage just
  to hold itself up; that's kG. (The lesson's `IntakeArm` sets `kG = Volts.of(0.35)`.)
- **kS** — volts to overcome *static friction* (the little shove to get moving).

Feedforward does the bulk of the work; feedback (the PID part) just cleans up what the prediction
missed.

---

## 3. PID

**PID** combines three responses to the error:

- **P (proportional)** — push harder the *bigger* the error. Gain **kP**. Too small → sluggish;
  too big → overshoot and oscillation.
- **I (integral)** — if a small error *persists*, accumulate it and push a bit more to erase it.
  Gain **kI**. Often left at 0.
- **D (derivative)** — react to how *fast* the error is changing, to damp overshoot. Gain **kD**.
  Often 0 for velocity control.

For a **flywheel** (velocity control), feedforward `kV` does almost everything and **kP** just
trims the residual — so the lesson's `IntakeRoller` and `Shooter` set only `kP` and `kV`:

```java
s.controlStyle = ControlStyle.PROFILED_PID;
s.kP = 0.15;   // trims the leftover error
s.kV = 0.115;  // predicts most of the voltage (theoretical ≈ 12 / free-speed-rot-per-sec ≈ 0.12)
```

PID is **onboard** here — it runs on the motor controller itself (a TalonFX), at very high rate.
You pick the gains, usually by starting from the theoretical kV and nudging kP until it's crisp.

**The mental model:** *kV gets you to the right speed; kP keeps you there.*

---

## 4. LQR

**LQR** (Linear-Quadratic Regulator) is a more advanced, **model-based** controller. Instead of
hand-picking gains, you give it a *physical model* — the mass, length, gearing, and motor — and
it computes the optimal gains for you. You only express *what you care about*: how tightly to hold
position vs. how much control effort to spend.

The lesson's `IntakeArm` uses LQR — and notice there are **no kP/kD to tune**:

```java
// controlStyle defaults to LQR — nothing to set
s.motorModel = DCMotor.getKrakenX60(1);
s.gearReductionStages = new double[] {10, 5}; // 50:1
s.length = Meters.of(0.5);
s.mass = Kilograms.of(3.0);
s.kG = Volts.of(0.35);     // still need the gravity feedforward
```

Because the controller is built from the physics, getting the *physical numbers* roughly right is
what matters — not fiddling with gains. LQR runs on the roboRIO each 20 ms cycle.

---

## 5. Which to use?

| | LQR | PID (+ feedforward) |
|---|---|---|
| You provide | physical model (mass, length, gearing, motor) | gains (kP, kV, …) |
| Tuning | little — get the physics right | hand-tune the gains |
| Runs on | roboRIO | onboard the motor controller |
| Great for | mechanisms with known geometry (arm, elevator) | simple velocity loops (flywheels); quick hand-tuning |

There's no single "right" answer — they're two valid tools. The lesson deliberately uses **one of
each** so you see both: LQR holds the arm, PID spins the wheels.

---

## 6. "At speed" / "at angle" tolerances

Closed-loop control never hits the target *exactly*, so you check whether you're **close enough**:

```java
shooter.isAtSpeed(Shooter.SHOOT_RPM, Shooter.RPM_TOLERANCE)   // within 150 RPM?
arm.isAtAngle(IntakeArm.DEPLOY_ANGLE, IntakeArm.ANGLE_TOLERANCE) // within 5°?
```

The lesson waits on `isAtSpeed` before firing — don't launch until the wheel is actually up to
speed. These return **Triggers**, which plug straight into command composition (see
[Command-based](command-based.md#6-composing-commands)).

---

### Back to the lesson

- [Module 2b — Add the arm (and meet LQR)](../intake-flywheel-lesson.md#step-2b--add-the-arm-and-meet-lqr)
- [Module 3 — Add the two flywheels (and meet PID)](../intake-flywheel-lesson.md#module-3--add-the-two-flywheels-and-meet-pid)
- Going further: the project's [mechanisms guide](../../mechanisms.md) and `/tune-mechanism`.
