# Deep Dive: Java Basics

> Background for the [intake + dual-flywheel lesson](../intake-flywheel-lesson.md). Read this if
> words like *class*, *object*, `extends`, or *lambda* are new. Every example here is taken from
> the lesson's own code, so it does double duty.

You do **not** need to memorize Java to do the lesson. You need a handful of ideas. This page
explains them one at a time.

---

## 1. A program is made of statements

A statement is one instruction, ended by a semicolon:

```java
int wheelCount = 4;          // make a whole number named wheelCount, set it to 4
double kP = 0.15;            // a decimal number
boolean deployed = false;    // a true/false value
String name = "IntakeArm";   // text
```

`int`, `double`, `boolean`, and `String` are **types** — they say what kind of value a name
holds. The compiler uses types to catch mistakes before the robot ever runs (you can't
accidentally put text where a number belongs).

---

## 2. Classes are blueprints; objects are the things you build

A **class** describes what something *is* and what it can *do*. An **object** is one real thing
built from that blueprint with the keyword `new`.

```java
public class Shooter {        // the blueprint
  // ...
}

Shooter shooter = new Shooter();   // one real Shooter object
```

Think of the class `Shooter` as the *design* of a flywheel, and `new Shooter()` as *manufacturing
one*. You can build many objects from one class.

Inside a class are **fields** (its data) and **methods** (its actions):

```java
public class Shooter extends Flywheel {
  public static final int CAN_ID = 43;   // a field

  public Shooter() {                      // a special method: the constructor
    super(settings());
  }

  private static Settings settings() {    // a method that returns settings
    ...
  }
}
```

- The **constructor** (`public Shooter()`) runs once, when you write `new Shooter()`. Its job is
  to set the object up.
- A **method** is a named block of code you can call, optionally returning a value.

---

## 3. `public`, `private`, and `static`

- **`public`** — other classes may use this. **`private`** — only this class may. Hiding details
  with `private` keeps things tidy: callers use a small public surface and don't depend on the
  internals.
- **`static`** belongs to the *class itself*, not to any one object. `IntakeArm.CAN_ID` is the
  same for everyone — there's one shared value, and you read it through the class name, no object
  needed. (`final` just means "can't be reassigned" — a constant.)

```java
public static final int CAN_ID = 41;   // one shared constant for all IntakeArms
... IntakeArm.CAN_ID ...                // read it via the class name
```

Non-static fields belong to each object: every `Shooter` has its own speed.

---

## 4. Packages and imports

A **package** is a namespace — a folder for classes that keeps names from colliding. This project
has three you'll meet:

| Package | What lives there |
|---|---|
| `frc.robot` | **Your** robot code (the lesson builds here) |
| `org.frc5010.common` | The reusable **library** (Arm, Flywheel, LedStripSegments…) |
| `org.frc5010.examples` | The library's **demos** (we don't depend on these) |

The first line of every file declares its package:

```java
package frc.robot.subsystems;
```

To use a class from another package, you **import** it:

```java
import org.frc5010.common.mechanisms.Arm;   // now we can write "Arm" instead of the full name
```

Your editor adds imports automatically (a "quick fix" when it sees an unknown name).

---

## 5. Inheritance: `extends` and `super`

**Inheritance** lets one class build on another. `class IntakeArm extends Arm` means "an
`IntakeArm` **is a kind of** `Arm`" — it gets everything `Arm` can do for free, and you only add
or change what's specific to your robot.

```java
public class IntakeArm extends Arm {
  public IntakeArm() {
    super(settings());   // call the Arm constructor with our settings
  }
}
```

- `Arm` is the **superclass** (or parent); `IntakeArm` is the **subclass** (or child).
- `super(...)` calls the parent's constructor. Here it hands `Arm` the configuration it needs.
- The whole lesson is built this way: you write *small* subclasses (`IntakeArm`, `Shooter`,
  `FuelHandler`, `StatusLeds`, `LessonRobot`) that fill in details, while the library parents do
  the heavy lifting.

### Overriding

A subclass can **override** a parent method — replace its behavior — by repeating its signature
and marking it `@Override`:

```java
@Override
public void periodic() {     // FuelHandler's own version of periodic()
  super.periodic();          // run the parent's version first...
  // ...then add our own behavior (drawing Fuel on the field)
}
```

`@Override` is a safety check: the compiler errors if there's no matching parent method to
override (so a typo can't silently create a brand-new method).

---

## 6. Lambdas and method references

Sometimes you want to pass *behavior* — a little function — into another object, so it can call
back later. Java writes that two ways:

```java
() -> fuelHandler.getHeldFuel() > 0     // a lambda: "a function that returns true if held > 0"
drive::getPose                          // a method reference: "the function drive.getPose"
```

Both are tiny functions you hand off. In the lesson you pass `drive::getPose` to `FuelHandler` so
it can ask for the robot's position *whenever it fires* (not just once). And `StatusLeds` takes
several of these so it can ask the `FuelHandler` for each device's status every cycle:

```java
new StatusLeds(9,
    fuelHandler::isIntakeExtended,   // "ask: is the intake out?"
    fuelHandler::isRollerSpinning,   // "ask: is the roller spinning?"
    ...);
```

The receiving type for these is usually a **functional interface** — `Supplier<T>` (gives a
value), `BooleanSupplier` (gives a true/false), `Runnable` (does something, returns nothing). You
don't have to create classes for them; a lambda or `::` is enough.

---

## 7. A few conveniences you'll see

- **`var`** — lets the compiler figure out the type: `var s = new Settings();` is the same as
  `Settings s = new Settings();`. Use it when the type is obvious from the right-hand side.
- **Enums** — a fixed set of named choices. `ControlStyle.LQR` and `ControlStyle.PROFILED_PID`
  are the two values of the `ControlStyle` enum.
- **Comments** — `// like this` or `/* ... */`; the computer ignores them, humans read them.

---

## 8. When something won't compile

The compiler prints the file, line, and a message. The most common beginner errors:

| Message | Usually means |
|---|---|
| `cannot find symbol` | A typo, or a missing `import` |
| `; expected` | You forgot a semicolon (often on the line *above*) |
| `incompatible types` | You put one type where another was expected (e.g. a number where a unit goes — see [Units](units.md)) |
| `method does not override...` | `@Override` on something that doesn't match a parent method |

Read the **first** error first — later ones are often just fallout from it.

---

### Further reading (external)

Free, beginner-friendly W3Schools Java tutorials for each idea above:

- [Java Intro](https://www.w3schools.com/java/java_intro.asp) ·
  [Syntax](https://www.w3schools.com/java/java_syntax.asp) ·
  [Variables](https://www.w3schools.com/java/java_variables.asp) ·
  [Data Types](https://www.w3schools.com/java/java_data_types.asp)
- [Classes & Objects](https://www.w3schools.com/java/java_classes.asp) ·
  [Class Attributes](https://www.w3schools.com/java/java_class_attributes.asp) ·
  [Class Methods](https://www.w3schools.com/java/java_class_methods.asp) ·
  [Constructors](https://www.w3schools.com/java/java_constructors.asp)
- [Modifiers (`public`/`private`/`static`/`final`)](https://www.w3schools.com/java/java_modifiers.asp) ·
  [Packages](https://www.w3schools.com/java/java_packages.asp)
- [Inheritance (`extends`)](https://www.w3schools.com/java/java_inheritance.asp) ·
  [Methods](https://www.w3schools.com/java/java_methods.asp)
- [Lambda expressions](https://www.w3schools.com/java/java_lambda.asp) ·
  [Enums](https://www.w3schools.com/java/java_enums.asp)

---

### Back to the lesson

- [Module 1 — Java & command-based ideas](../intake-flywheel-lesson.md#module-1--java-and-command-based-ideas-youll-use)
- Related deep dives: [WPILib Units](units.md) · [Command-based programming](command-based.md)
