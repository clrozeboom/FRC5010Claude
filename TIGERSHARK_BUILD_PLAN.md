# Lesson Plan: Building the TigerShark Robot

> **Format:** This is a guided walkthrough. Each lesson has a goal, the concepts you
> need to understand *before* writing code, the work itself, and a checkpoint to
> verify before moving on. Tell Claude when you hit an issue at a checkpoint and
> we'll revise the lesson before continuing.

## Course overview

**You are building:** a new FRC robot configuration named "TigerShark" — a swerve
drivetrain (Krakens, configured via Phoenix Tuner X) plus a two-motor elevator on a
single gearbox. Vision gets a placeholder PhotonVision camera. LEDs stay in but get
rewritten to be independent of the demo intake.

**What you'll learn along the way:**

- How this framework separates "hardware constants" (a `RobotProfile`) from
  "robot behaviour" (a `SwerveRobotContainer` subclass) from "the wiring shell"
  (`RobotContainer.java`).
- How TalonFX swerve REAL-mode wiring differs from sim wiring, and why
  `SwerveFactory.build()` deliberately throws on the hardware path.
- How a two-motor mechanism is configured (it's one extra field — but with two
  gotchas attached).
- How LQR vs. profiled-PID control choices interact with what you actually know
  about the mechanism.
- How button bindings flow through `configureBindings()` and why subsystem
  lifecycle (`registerMechanism`) matters for tests.

**Course structure:**

| # | Lesson | Goal |
|---|---|---|
| 0 | Branch setup | Isolate your work from `main` |
| 1 | TunerConstants intake | Understand what TunerX produces |
| 2 | TigerSharkRobotProfile | Describe the drivetrain to the framework |
| 3 | TigerSharkElevator | Configure the two-motor elevator |
| 4 | TigerSharkLeds | Decouple LEDs from the demo intake |
| 5 | TigerSharkRobot | Wire subsystems into button bindings |
| 6 | RobotContainer swap | Point the entry point at TigerShark |
| 7 | Sim verification | Prove it works before hardware |
| 8 | Hand-off (preview) | What comes next: tuning, calibration, deploy |

**Files you'll create** (all under `src/main/java/frc/robot/tigershark/`):

- `TunerConstants.java`, `TigerSharkRobotProfile.java`, `TigerSharkElevator.java`,
  `TigerSharkLeds.java`, `TigerSharkRobot.java`

**Files you'll edit:** just `src/main/java/frc/robot/RobotContainer.java` (two lines).

**Files you'll leave alone but use as reference:** everything in
`org/frc5010/examples/` and `org/frc5010/examples/mechanisms/Example*.java`.

---

## Lesson 0 — Branch setup

### Goal
Get onto a feature branch so your robot's code can evolve separately from the
upstream library.

### Concept
The `main` branch of this repo is the **library**. Your robot is downstream of it.
Doing your team's work on a branch means you can keep pulling library improvements
without merge fights and without polluting `main` with team-specific constants.

### Work
```powershell
git checkout -b tigershark
```

### Checkpoint
- `git status` shows branch `tigershark`, working tree clean.
- `git log --oneline -1` shows the same commit that's at the tip of `main`.

### Self-check question
*If your team later wants to test your TigerShark code on a second physical robot
chassis, would you create another branch off `tigershark`, or off `main`? Why?*

---

## Lesson 1 — TunerConstants intake

### Goal
Get the Phoenix Tuner X export into the project as untouchable generated code.

### Concept

Phoenix Tuner X's Swerve Project Generator produces **one file**:
`TunerConstants.java`. It contains:

- The four module configurations as `SwerveModuleConstants` objects named
  `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`.
- All drive/steer motor and CANcoder IDs.
- The Pigeon 2 (or NavX) gyro ID.
- The CAN bus name.
- The wheel radius, gear ratios, and TunerX-calibrated gains.

Why "untouchable": when you re-tune the robot (new offsets after a crash, a new
gearbox, new wheels), you re-run TunerX and **overwrite** this file. Edits you
made by hand get wiped out. Put your hand-edits in
`TigerSharkRobotProfile` (Lesson 2) instead.

### Work
1. Paste the TunerX-generated file into
   `src/main/java/frc/robot/tigershark/TunerConstants.java`.
2. Change the package declaration at the top to
   `package frc.robot.tigershark;` if TunerX put it elsewhere.
3. **Read it once** before moving on. Look for these names (you'll cite them later):
   `kFrontLeftDriveMotorId`, `kFrontLeftSteerMotorId`, `kFrontLeftEncoderId`,
   `kFrontLeftEncoderOffset`, `kWheelRadius` (or similar),
   `kDriveGearRatio`, `kSteerGearRatio`, `kCANBus`, `FrontLeft`,
   `FrontRight`, `BackLeft`, `BackRight`.

### Checkpoint
- The file compiles (you can verify in your IDE without running Gradle).
- You can answer the questions below from looking at it.

### Self-check questions
- *What CAN ID is your Front-Left drive motor on?*
- *Is `kCANBus` set to `""`, or to a CANivore device name?*
- *What's the value of `kWheelRadius` (or the wheel radius constant), in inches
  and metres?*

---

## Lesson 2 — TigerSharkRobotProfile

### Goal
Describe TigerShark's swerve drivetrain to the framework: physical geometry, CAN
IDs, gyro choice, starting pose, and the REAL/SIM dispatch in `createDrive()`.

### Concepts

**Three reasons `SwerveConstants` exists:**

1. **REAL mode** uses it to dispatch to the right `ModuleIO` (TalonFX vs.
   Spark/Talon) and to know the right gyro class.
2. **SIM mode** uses it to build an IronMaple physics model of *your specific
   robot*. Mass, bumper size, and wheel geometry shape how the sim accelerates and
   collides. Lie about the mass → sim is fast, real robot is slow.
3. **Motion control** uses `maxLinearSpeed` etc. to clamp teleop and plan
   trajectories.

**Why the factory throws in REAL mode (CLAUDE.md gotcha #8):**
`SwerveFactory.build()` can wire up sim-mode TalonFX because the sim path uses
synthesized motor specs. For the real path it needs your *team's*
`SwerveModuleConstants` — gear ratios, gains, encoder offsets — and it has no way
to know those. So it throws, and your `createDrive()` wires the IO classes
manually. The throw isn't a bug; it's a "you need to do this part" sign.

**Starting pose:** you write only the blue-alliance start. The framework mirrors it
for red automatically (`SwerveRobotContainer.getAllianceStartPose()` flips x across
the field centre and rotates 180°).

### Work

Path: `src/main/java/frc/robot/tigershark/TigerSharkRobotProfile.java`

1. Copy `src/main/java/org/frc5010/examples/ExampleRobotProfile.java` to the new
   location. Change the package to `frc.robot.tigershark` and the class name to
   `TigerSharkRobotProfile`.

2. Rewrite the `CONSTANTS` builder with TigerShark's actual numbers:

   ```java
   private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
       .moduleType(ModuleType.TALON_FX)
       .gyroType(GyroType.PIGEON2)             // change to NAVX if applicable
       .gyroCanId(/* from TunerConstants.kPigeonId */)
       .trackWidth(Inches.of(/* MEASURED left-to-right wheel centres */))
       .wheelBase(Inches.of(/* MEASURED front-to-back wheel centres */))
       .wheelRadius(Inches.of(/* TunerX worn-wheel value */))
       .maxLinearSpeed(MetersPerSecond.of(/* measured top speed */))
       .maxAngularSpeed(RadiansPerSecond.of(/* measured top spin rate */))
       .robotMass(Pounds.of(/* weighed with bumpers + battery */))
       .bumperLength(Inches.of(/* outside to outside, front-to-back */))
       .bumperWidth(Inches.of(/* outside to outside, side-to-side */))
       .frontLeftIds(/* drive, steer, encoder — from TunerConstants */)
       .frontRightIds(/* drive, steer, encoder */)
       .backLeftIds(/* drive, steer, encoder */)
       .backRightIds(/* drive, steer, encoder */)
       .canBusName("")                         // or "canivore" — match TunerConstants.kCANBus
       .odometryFrequency(Hertz.of(100))       // 250 for CANivore, 100 for RIO bus
       .build();
   ```

   **Why duplicate the CAN IDs in two places** (here AND in `TunerConstants`)?
   The framework needs them for sim-mode IO selection independent of the TunerX
   record. If they disagree, sim and real diverge silently — make them match.

3. Set `BLUE_START` to your team's blue-alliance starting position for the
   current game.

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
   and `frc.robot.tigershark.TunerConstants`.

5. **Skip `createVision()` for now.** The base `RobotProfile.createVision()`
   already returns `null`, which disables vision entirely. Do **not** override
   it until you have a PhotonVision coprocessor physically connected and
   configured. Overriding it with a missing camera causes a stream of
   `PhotonVision coprocessor not found` errors every loop cycle and wastes CPU.

   When you are ready to add vision later, restore the override with:
   - The real `FRONT_CAM_TRANSFORM` for TigerShark's camera mounting position.
   - The correct camera name matching what is set in PhotonVision's web UI.

   Remove (or do not add) these imports until then — the compiler will flag them
   unused and they clutter the file:
   `CameraConfig`, `Vision`, `VisionFactory`, `AprilTagFieldLayout`,
   `AprilTagFields`, `Rotation3d`, `Transform3d`, `Translation3d`.

### Checkpoint
- File compiles in your IDE — no red squigglies.
- The CAN IDs in `frontLeftIds(...)` match `TunerConstants.kFrontLeft*` exactly.
- `BLUE_START` is set to a real field position, not the placeholder `(1.5, 2.0)`.
- `createVision()` is **not** overridden (no camera yet).

### Self-check questions
- *Why does the framework need `robotMass` for the simulator? What goes wrong if
  you set it to half its real value?*
- *If you swapped two pairs of `*Ids(...)` calls (e.g. front-left and back-right),
  what would the simulator show when you push forward?*
- *Why is `wheelRadius` measured rather than taken from the spec sheet?*

---

## Lesson 3 — TigerSharkElevator

### Goal
Describe the two-motor elevator: physics, geometry, and control style. Don't
write any control logic — that's all in the common `Elevator` class.

### Concepts

**Master + follower on one gearbox.** Two motors, one drum, one effective output.
The framework handles this via two fields in `Settings`:

- `canId` — the lead motor.
- `followerCanId` — the follower. The library configures the second TalonFX as a
  Phoenix follower of the first, so software-side it behaves like one motor.

**The plant gotcha** — `motorModel`. This is the simulated physics model. It
defaults to `DCMotor.getKrakenX60(1)` (one motor of torque/free-speed). If you
have two motors mechanically combined, you must set
`DCMotor.getKrakenX60(2)`. The `2` doubles torque and current capacity in the
plant. If you forget, sim thinks your elevator is half as strong as it is, your
LQR plant is wrong, and your tuning will be wrong.

**`followerOpposed`** — set true when the follower is mounted physically facing
the opposite way from the lead (very common — gearboxes often have motors
back-to-back). It flips the follower's voltage direction so they drive the same
direction mechanically. Get this wrong and the motors will fight each other and
trip the breaker.

**Profile cruise velocity sanity check (gotcha #12).** The trapezoid profile asks
the elevator to hit `maxVelocity` then cruise. If `maxVelocity` is higher than
what the motors can physically reach, the profile runs away from the mechanism,
the error grows, and the controller saturates with overshoot. Check:
```
freeSpeedRotorRps   = 100              (Kraken X60)
freeSpeedDrumRps    = 100 / gearing
freeSpeedLinearMps  = freeSpeedDrumRps × drumCircumference (m)
```
Pick `maxVelocity` ≲ 80% of `freeSpeedLinearMps`.

**Control style.**
- `ControlStyle.LQR` (default): the framework builds an LQR controller from
  physical parameters (mass, gearing, motor model). Best when you trust those
  numbers. No `kP/kI/kD` to tune — you tune the LQR weights `qelms/relms`
  instead.
- `ControlStyle.PROFILED_PID`: a trapezoid profile feeding MotionMagic on the
  TalonFX. You provide `kP`, `kV`, optionally `kI`/`kD`. Pick this when the mass
  is uncertain, when you want hand-tuneable gains, or when LQR convergence is
  flaky.

For TigerShark we'll start with **LQR** because (a) you know your mass and
gearing, and (b) the example elevator is LQR and you can mirror its structure.

**`kG` (gravity feedforward).** The volts needed to hold the carriage stationary
against gravity. Sim works from any reasonable guess (you can compute it from
mass and gearing); on the real robot, characterize it with `sysId()` later
(`/tune-mechanism`).

### Work

Path: `src/main/java/frc/robot/tigershark/TigerSharkElevator.java`

1. Copy `src/main/java/org/frc5010/examples/mechanisms/ExampleElevator.java` into the new
   package.

2. Change the package to `frc.robot.tigershark` and rename the class to
   `TigerSharkElevator`.

3. Rewrite `settings()`:

   ```java
   var s = new Settings();
   s.name = "TigerSharkElevator";
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

   s.visualPose3d = new Pose3d(
       /* x, y, z mount point of the lead motor in robot frame, metres */,
       Rotation3d.kZero);
   return s;
   ```

4. **Run the cruise-velocity sanity check** before moving on. Write the three
   numbers (`freeSpeedDrumRps`, `freeSpeedLinearMps`, your chosen
   `maxVelocity`) in a comment above the `s.maxVelocity` line so anyone reading
   can verify it later.

5. **Expose an `isMoving()` helper** that the LED class will subscribe to. The
   base `Elevator` already publishes `getVelocity()` (returns a `LinearVelocity`),
   so the helper is one line plus a threshold constant:

   ```java
   private static final double MOVING_THRESHOLD_MPS = 0.02;

   /** True when the carriage is moving fast enough to count as "in motion". */
   public boolean isMoving() {
     return Math.abs(getVelocity().in(MetersPerSecond)) > MOVING_THRESHOLD_MPS;
   }
   ```

   Why a threshold rather than `!= 0`: gear lash, sensor noise, and tiny LQR
   correction commands mean a stopped carriage still reports ~mm/s of velocity.
   A 2 cm/s dead-band keeps the "moving" LED state honest. Also expose
   `SCORING_HEIGHT` as a public `Distance` constant — `TigerSharkRobot` will
   reference it for a button binding.

### Checkpoint
- File compiles.
- `s.motorModel` is `DCMotor.getKrakenX60(2)` — not `(1)`.
- The cruise-velocity sanity check is documented in a comment above
  `maxVelocity`.
- `isMoving()` and `SCORING_HEIGHT` are both public on the class.

### Self-check questions
- *If `followerOpposed` is wrong, what symptom do you see at power-on?*
- *If `motorModel` is `getKrakenX60(1)` but you actually have two motors, what
  happens to your LQR tuning when you take it to the real robot?*
- *Compute: gearing 20:1, drum circumference 3.456" (22-tooth #25 chain). What's
  the maximum possible linear speed of the carriage? What's a safe `maxVelocity`
  to pick?*

---

## Lesson 4 — TigerSharkLeds

### Goal
Build LEDs that respond to TigerShark-relevant state (mechanism, drive) — not the
demo intake state `DemoLeds` was wired to.

### Concept

`DemoLeds` couples to `DemoIntake` directly — it calls
`DemoIntake.isInAllianceZone(pose)` and has a `notifyShot()` method. That made
sense for the demo because the demo robot has an intake. TigerShark doesn't (yet),
so we re-implement using **generic `Supplier`/`BooleanSupplier` parameters** —
that way the LED class only knows about LED state, not about which subsystem
produced it.

The base class `LedStripSegments` does all the heavy lifting:

- Owns the `AddressableLED` PWM device.
- Lets you split the strip into named `Segment`s with their own `LEDPattern`.
- Supports a whole-strip override (for disabled-state animations).

You're writing a thin `periodic()` that maps `(robot state) → (LED pattern)`.

### Work

Path: `src/main/java/frc/robot/tigershark/TigerSharkLeds.java`

1. Copy `src/main/java/org/frc5010/examples/DemoLeds.java` to the new location.

2. Change the package, rename to `TigerSharkLeds`, and **rip out anything that
   references `DemoIntake`** — including the `intakeExtended` param,
   `notifyShot()`, and the `isInAllianceZone` call.

3. Decide on a starter mapping. A reasonable first cut:

   | Robot state | Pattern |
   |---|---|
   | Disabled, never enabled | Solid alliance colour (or green if unknown) |
   | Disabled, previously enabled | Scrolling rainbow |
   | Enabled, elevator moving | Red Larson scanner across the whole strip |
   | Enabled, elevator idle at goal | Solid alliance colour |

4. Accept the state as a `BooleanSupplier elevatorMoving` constructor argument.
   Don't reach into the elevator class from inside the LED class — that's how
   coupling grows back. The robot container will pass `() -> elevator.isMoving()`
   (or whatever you decide).

5. Keep the `periodic()` skeleton from `DemoLeds`:
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
- All inputs come through the constructor as `Supplier`/`BooleanSupplier`.

### Self-check question
- *Why pass `BooleanSupplier elevatorMoving` instead of `TigerSharkElevator
  elevator`? What's the downside of giving the LED class the full elevator
  reference?*

---

## Lesson 5 — TigerSharkRobot

### Goal
Wire the drive (already built by the profile), the elevator, and the LEDs into a
`SwerveRobotContainer` subclass. Bind controller buttons.

### Concepts

**`SwerveRobotContainer` lifecycle in this codebase:**

1. Its constructor takes the `RobotProfile` (you build via `selectProfile(FQN)`).
2. The constructor calls `configureBindings()` **before your subclass
   constructor body runs**. That's why `ExampleRobot` initialises
   subsystems *inside* `configureBindings()` — fields set in the subclass
   constructor body would be `null` at the moment `configureBindings()` reads
   them.
3. `buildAutos()` runs lazily, on the first scheduler tick, after construction
   is fully complete. Safe for any subsystem reference.
4. `registerMechanism(closeable)` registers a teardown callback. The smoke test
   uses these to free CAN IDs and PWM handles between tests. Skipping it makes
   the test suite flaky.

**Goal commands never finish.** `elevator.goToHeight(Meters.of(0.6))` holds the
goal forever (until interrupted by another command on the elevator subsystem).
Bind separate buttons with `.onTrue(...)` for each preset — don't chain with
`.andThen` thinking the first will release.

**`selectProfile(...)` is reflection.** You pass the fully-qualified class name
as a string. Typos fail at runtime, not compile time. Be careful.

### Work

Path: `src/main/java/frc/robot/tigershark/TigerSharkRobot.java`

1. Copy `src/main/java/org/frc5010/examples/ExampleRobot.java` into the new package. Rename class to
   `TigerSharkRobot`.

2. **Strip out** all the demo-mechanism construction
   (`configureDemoMechanisms()`, `demoElevator`, the imports of all the
   `Example*` mechanisms, `DemoIntake`, the X button binding for mid/start
   positions). Keep the basic class shape (extends `SwerveRobotContainer`, has
   `buildAutos()` and `configureBindings()`).

3. Pass your profile's FQN to `super`:
   ```java
   public TigerSharkRobot() {
     super(SwerveRobotContainer.selectProfile("frc.robot.tigershark.TigerSharkRobotProfile"));
   }
   ```

4. Strip `buildAutos()` to just `None`:
   ```java
   @Override
   protected void buildAutos() {
     addAuto("None", Commands.none());
   }
   ```
   You'll add real autos later — see `docs/auto.md` and
   `org/frc5010/examples/AutoRoutines.java` for the patterns when you're ready.

5. Write `configureBindings()`:
   ```java
   @Override
   protected void configureBindings() {
     super.configureBindings();              // wires drive + standard buttons — don't skip

     // ⚠️ Gotcha — replace the default keyboard drive with proper Xbox axes.
     // The base configureBindings() maps rotation to axis(2), which is the LEFT
     // TRIGGER on an Xbox controller, not the right stick. Override the drive
     // default command here, after calling super, using the correct named axes.
     JoystickAxis forward  = controller.leftY().negate().deadzone(0.05).power(2.0);
     JoystickAxis strafe   = controller.leftX().negate().deadzone(0.05).power(2.0);
     JoystickAxis rotation = controller.rightX().negate().deadzone(0.10);
     DriveVector translate = DriveVector.of(forward, strafe).unitCircle();

     drive.setDefaultCommand(
         Commands.run(
             () -> {
               double flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                   ? -1.0 : 1.0;
               Translation2d xy = translate.get();
               drive.runVelocityFieldRelative(new ChassisSpeeds(
                   flip * xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                   flip * xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                   rotation.getAsDouble() * drive.getMaxAngularSpeed().in(RadiansPerSecond)));
             },
             drive
         ).withName("XboxDrive")
     );

     elevator = new TigerSharkElevator();
     registerMechanism(elevator::close);

     // Three preset heights — low / scoring / max.
     // SCORING_HEIGHT is the public constant you added on TigerSharkElevator in
     // Lesson 3; maxHeight comes from settings so it can't drift from the
     // mechanism's actual soft limit.
     controller.a().onTrue(elevator.goToHeight(Meters.of(0.0)));
     controller.b().onTrue(elevator.goToHeight(TigerSharkElevator.SCORING_HEIGHT));
     controller.y().onTrue(elevator.goToHeight(elevator.getSettings().maxHeight));

     // LEDs are real hardware on a PWM port, so create them in both real and sim
     // (unlike DemoLeds which was sim-only because DemoIntake is sim-only)
     leds = new TigerSharkLeds(LED_PWM_PORT, elevator::isMoving);
     registerMechanism(leds::close);
   }
   ```

   Add these imports for the drive command:
   ```java
   import static edu.wpi.first.units.Units.MetersPerSecond;
   import static edu.wpi.first.units.Units.RadiansPerSecond;
   import org.frc5010.common.input.DriveVector;
   import org.frc5010.common.input.JoystickAxis;
   import edu.wpi.first.math.geometry.Translation2d;
   import edu.wpi.first.math.kinematics.ChassisSpeeds;
   import edu.wpi.first.wpilibj.DriverStation;
   import edu.wpi.first.wpilibj.DriverStation.Alliance;
   ```

6. Declare the fields and the PWM port constant:
   ```java
   private static final int LED_PWM_PORT = 9;
   private TigerSharkElevator elevator;
   private TigerSharkLeds leds;
   ```

### Checkpoint
- File compiles.
- `super.configureBindings()` is the first line of `configureBindings()`.
- The `XboxDrive` default command uses `controller.leftY()`, `controller.leftX()`,
  and `controller.rightX()` — **not** `axis(2)` for rotation.
- Both `elevator` and `leds` are registered via `registerMechanism(...)`.
- No references to `Example*` classes, `DemoIntake`, or `DemoLeds`.

### Self-check questions
- *Why must subsystems be initialised inside `configureBindings()` rather than
  in the constructor body?*
- *What happens if you skip `super.configureBindings()`?*
- *If you write `controller.a().onTrue(elevator.goToHeight(low).andThen(
  elevator.goToHeight(high)))`, what actually happens when the driver presses
  A? (Trick question — think about the "goal commands never finish" rule.)*
- *Why does the base `SwerveRobotContainer` use `axis(2)` for rotation? What
  device is typically on axis 2 in simulation, and why does that not work for
  a real Xbox controller on a competition robot?*

---

## Lesson 6 — RobotContainer swap

### Goal
Point the entry point at TigerShark. This is the only edit to existing code.

### Concept
`frc/robot/Robot.java` instantiates `frc.robot.RobotContainer`. That class is a
thin shell that constructs *one* team container and delegates to it. Swapping
the team container in this one file is how you switch between robots without
touching framework code.

### Work

Edit `src/main/java/frc/robot/RobotContainer.java`:

```java
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tigershark.TigerSharkRobot;        // ← changed import

public class RobotContainer {
  private final TigerSharkRobot robot;              // ← changed type

  public RobotContainer() {
    robot = new TigerSharkRobot();                  // ← changed construction
  }

  public Command getAutonomousCommand() { return robot.getAutonomousCommand(); }
  public void resetToAllianceStart()    { robot.resetToAllianceStart(); }
}
```

### Checkpoint
- Project compiles end-to-end (try `.\gradlew.bat compileJava` in PowerShell).
- No remaining references to `ExampleRobot` in this file.
- `ExampleRobot.java` and `ExampleRobotProfile.java` still exist untouched —
  they're your reference, not dead code.

### Self-check question
- *If a teammate later wants to test a different swerve config without losing
  TigerShark, what's the minimum change they'd make? Could two robots coexist
  in the codebase?*

---

## Lesson 7 — Sim verification

### Goal
Prove the whole stack works in the simulator before sending it to a real robot.

### Concepts

`./gradlew test` runs the test pyramid layers 1–3 (see `docs/testing.md`). The
smoke test will construct your `TigerSharkRobot` (because `RobotContainer` now
does) and fail loudly if any constant is missing, any CAN ID conflicts, or any
constructor throws.

`./gradlew simulateJava` launches Glass + Driver Station + (if `-PwebUI` is
passed) the web control panel. The robot starts disabled — click **Enable** in
the Driver Station panel.

What you're looking for:

- **Field2d** at SmartDashboard's main panel shows a bumper-sized rectangle at
  `BLUE_START`. Wrong starting pose → wrong constants.
- **SmartDashboard → `TigerSharkElevator/mechanism`** shows the side view of the
  elevator. Press A/B/Y on an Xbox controller and the carriage should track each
  preset smoothly. Overshoot or runaway → re-check the velocity sanity check
  (Lesson 3 concept).
- **AdvantageScope → `Mechanisms3d/TigerSharkElevator`** shows the 3D view. Both
  motor symbols (lead and follower mirror) should be visible.
- Driving with WASD or a controller stick — robot translates without spinning.
  If it spins, your module CAN IDs are crossed.

### Work
```powershell
.\gradlew.bat test
.\gradlew.bat simulateJava
```

### Checkpoint

| Check | Pass criterion |
|---|---|
| Tests pass | `BUILD SUCCESSFUL`, no `RobotContainerDelegationTest` failure (the test that constructs `RobotContainer` → your `TigerSharkRobot`). The renamed `ExampleRobotSmokeTest` still runs against the demo robot independently. |
| Field2d at start pose | Robot rectangle visible at `BLUE_START` |
| Drive | WASD or stick moves robot without unwanted yaw |
| Elevator A | Carriage tracks to 0.0 m smoothly |
| Elevator B | Tracks to 0.6 m |
| Elevator Y | Tracks to 1.2 m |
| LEDs | Larson while elevator moves, alliance solid when idle |

### Self-check / debugging questions
- *If the test fails with `NullPointerException` in `TigerSharkRobot`, what's
  the most common cause given the lifecycle in Lesson 5?*
- *If the elevator overshoots violently in sim, which Settings field is the
  first thing to check?*
- *If the robot spins instead of going straight, which constants are suspect?*

---

## Lessons learned on first hardware deploy

These issues were discovered when deploying TigerShark to the real roboRIO for
the first time. They are **not** caught in simulation. Add them to your mental
checklist before every new hardware deploy.

| Symptom | Root cause | Fix |
|---------|-----------|-----|
| Constant `PhotonVision coprocessor not found` errors filling the DS log | `createVision()` is overridden but the camera co-processor is not connected | Remove the `createVision()` override until the co-processor is physically installed and running; the base returns `null` which silently disables vision |
| Translation works, rotation does nothing | Base `configureBindings()` assigns rotation to `axis(2)` (left trigger on Xbox); right stick X is axis 4 | Override `drive.setDefaultCommand(...)` in `TigerSharkRobot.configureBindings()` after calling `super`, using `controller.rightX()` for rotation |

---

## Lesson 8 — Hand-off (preview, not for this branch)

These are the next playbooks you'll use *after* the TigerShark code structure is
proven in sim:

- **`/calibrate-drive`** — measure your real drive motors with SysId, update
  `TunerConstants` (or `DriveConstants`) with the calibrated gains.
- **`/tune-mechanism`** — characterize the elevator on the real robot:
  - `kG`, `kV`, `kA` via SysId quasistatic and dynamic tests.
  - LQR weights (`qelms` / `relms`) via the NetworkTables tuning panel.
- **`/diagnose-log`** — pull a `.wpilog` after match practice and look for
  anomaly flags (motor saturation, slip, vision dropouts).
- **Deployment** — `.\gradlew.bat deploy` to the roboRIO. Walk through the
  hardware checklist in `docs/student-setup.md` before driving for real.

---

## Files this lesson plan creates / edits

**New** (`src/main/java/frc/robot/tigershark/`):
- `TunerConstants.java`
- `TigerSharkRobotProfile.java`
- `TigerSharkElevator.java`
- `TigerSharkLeds.java`
- `TigerSharkRobot.java`

**Edited:**
- `src/main/java/frc/robot/RobotContainer.java`

**Untouched (reference):**
- Everything under `org/frc5010/examples/` and `org/frc5010/examples/mechanisms/`.

---

## How to use this plan with Claude

- Work through one lesson at a time.
- At each **Checkpoint**, run the listed checks. If something fails, paste the
  error (or describe the unexpected behaviour) to Claude — we'll revise that
  lesson before moving on.
- The **Self-check questions** are for your understanding. Answer them out loud
  before moving to the next lesson; if you're stumped, ask Claude.
- Don't skip ahead. Lesson 5 assumes Lesson 3's gotchas were internalised; the
  sim verification in Lesson 7 won't make sense without Lesson 2's profile.
