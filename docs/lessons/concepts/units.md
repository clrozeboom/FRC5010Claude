# Deep Dive: WPILib Units

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). Read this to
> understand `Degrees.of(110)`, `RPM.of(3000)`, and friends.

## 1. The problem units solve

Plain numbers don't say what they measure. Is `110` degrees or radians? Is `3000` RPM or
rotations per second? Mixing them up is one of the most common — and most expensive — bugs in
engineering (a NASA Mars orbiter was lost in 1999 to a pounds-vs-newtons mix-up).

WPILib fixes this by wrapping numbers in **typed units**. Instead of a bare `110`, you write
`Degrees.of(110)`. Now the value *carries its unit with it*, and the compiler won't let you put a
distance where an angle belongs.

---

## 2. Creating a measurement

The pattern is always `Unit.of(number)`:

```java
import static edu.wpi.first.units.Units.*;   // lets you write Degrees, RPM, Meters directly

Degrees.of(110)        // an angle
RPM.of(3000)           // an angular velocity (rotations per minute)
Meters.of(0.5)         // a distance
Inches.of(4)           // also a distance — different unit, same dimension
Kilograms.of(3.0)      // a mass
Volts.of(0.35)         // a voltage
DegreesPerSecond.of(220)  // an angular velocity (a speed of rotation)
```

That `import static` line is what lets you write `Degrees` instead of the long
`edu.wpi.first.units.Units.Degrees`. The lesson files all start with it.

---

## 3. Reading the number back

When you need the raw number (to do math, log it, or compare), ask for it **in a specific unit**
with `.in(...)`:

```java
double deg = arm.getAngle().in(Degrees);     // e.g. 110.0
double rpm = shooter.getSpeed().in(RPM);      // e.g. 2987.4
```

This is the safe moment to convert: `someDistance.in(Meters)` and `someDistance.in(Inches)` give
the same physical length expressed two ways. You ask for what you want; the library converts.

---

## 4. Dimensions vs. units

A **dimension** is *what* you're measuring; a **unit** is *how* you express it. The lesson uses
these dimensions (Java type → example units):

| Dimension (Java type) | Example units |
|---|---|
| `Angle` | `Degrees`, `Radians`, `Rotations` |
| `AngularVelocity` | `RPM`, `RotationsPerSecond`, `DegreesPerSecond`, `RadiansPerSecond` |
| `Distance` | `Meters`, `Inches` |
| `Mass` | `Kilograms`, `Pounds` |
| `Voltage` | `Volts` |
| `AngularAcceleration` | `DegreesPerSecondPerSecond` |

A method that wants an `Angle` will take `Degrees.of(...)` *or* `Radians.of(...)` — both are
angles. It will **refuse** `RPM.of(...)`, because that's a different dimension. That refusal,
caught at compile time, is the whole point.

---

## 5. Where you see this in the lesson

```java
public static final Angle DEPLOY_ANGLE = Degrees.of(110);          // IntakeArm
public static final AngularVelocity SHOOT_RPM = RPM.of(3000);      // Shooter
s.length = Meters.of(0.5);                                          // arm length
s.kG = Volts.of(0.35);                                             // gravity feedforward
arm.goToAngle(IntakeArm.DEPLOY_ANGLE);                              // pass a typed angle
```

Because `goToAngle` takes an `Angle`, you literally cannot call it with a speed by mistake.

---

## 6. A note for later

There's one place units **don't** go: the logging structs marked `@AutoLog` (deep in the
library's IO layer) must use plain `double` fields, not units. You won't touch those in this
lesson, but if you explore the mechanism internals later and see `double positionRot`, that's
why. Extract the raw value with `.in(...)` before storing it there.

---

### Back to the lesson

- [Module 1 — Java & command-based ideas](../intake-flywheel-lesson.md#module-1--java-and-command-based-ideas-youll-use)
- Related: [Java basics](java-basics.md) · the project's [configuration reference](../../configuration.md)
