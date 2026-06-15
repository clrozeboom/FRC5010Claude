# /new-mechanism — Add a TalonFX mechanism subsystem

Agent playbook for adding an elevator, arm, pivot/turret, flywheel/shooter,
double-jointed arm, or differential wrist using the TalonFX-native mechanism classes
in `org.frc5010.common.mechanisms`. Read [docs/mechanisms.md](../../docs/mechanisms.md)
first — especially the Gotchas section.

## Step 1 — Pick the mechanism type and control style

| Robot part | Class | Default controller |
|---|---|---|
| Elevator, climber (linear travel) | `Elevator` | ELEVATOR LQR (meters) |
| Arm with gravity (shoulder, intake pivot) | `Arm` | ARM LQR + kG·cos(θ) |
| Turret, hood, gravity-free wrist | `Pivot` | ARM LQR, no gravity FF |
| Shooter wheel, roller (velocity) | `Flywheel` | velocity LQR |
| Two-jointed arm (shoulder+elbow) | `DoubleJointedArm` | onboard MotionMagic per joint (only style) |
| Tilt+twist differential wrist | `DifferentialMechanism` | onboard MotionMagic per motor (only style) |

The four single-DOF classes also support `s.controlStyle = ControlStyle.PROFILED_PID`
(onboard MotionMagic / VelocityVoltage with kP/kI/kD + kS/kV/kG). Pick PROFILED_PID
when the team prefers hand-tunable gains or the mass/MOI model is uncertain; pick LQR
when the physical parameters are known (or characterized via SysId). Profiled
examples: `ExampleProfiledElevator/Arm/Turret/Shooter` (CAN 31–34).

## Step 2 — Gather the robot-specific numbers

Required before writing code (ask the user if unknown):
- TalonFX CAN ID(s). This library is TalonFX-only.
- Gear reduction stages rotor→mechanism (e.g. `{4, 3}` = 12:1).
- Elevator: drum/sprocket circumference, carriage mass, min/max/start height.
- Arm: length, mass, limits, starting angle.
- Pivot: MOI of the rotating assembly, limits, starting angle.
- Flywheel: wheel diameter + mass.
- **Profile cruise velocity must be below free speed ÷ gearing × circumference** —
  compute it and check, don't trust the user's wish speed.

## Step 3 — Create the subsystem in TEAM code (`org.frc5010.examples.mechanisms`)

Copy the matching `Example*` class from `src/main/java/org/frc5010/examples/mechanisms/` and
replace the constants. Keep the pattern: a thin class extending the common class
whose only content is a `settings()` method. **Never put control logic in team code**
— if something is missing, extend the common class in
`org.frc5010.common.mechanisms` instead.

`RobotMode.set(...)` must run before any mechanism is constructed (Robot.java already
does this; tests set `Mode.SIM` in their setup).

## Step 4 — Wire commands

The class exposes ready-made commands: `goToHeight`/`goToAngle`/`goToSpeed`
(never finish — bind to buttons or sequence with `.until(isAt...(...))`),
`setDutyCycle` for manual jog, `sysId()` for characterization (limit-guarded).
If the mechanism is created in a robot container, register it with
`registerMechanism(mech::close)` so tests can tear it down.

## Step 5 — Add a functional test

Add a test to `src/test/java/org/frc5010/examples/mechanisms/MechanismsFunctionalTest.java`,
copying its pump (`runScheduledFor`): synchronous `stepOneCycle()` is fine — there are
no Notifier threads — but every cycle must feed `DriverStationSim.notifyNewData()` +
`Unmanaged.feedEnable(...)` and sleep a few ms so the simulated TalonFX's real-time
device thread can process controls. Schedule the command, run ~4 sim-seconds, then
use `assertConverges`. Always `close()` the subsystem in a `finally` block to free
CAN IDs, and set/reset `RobotMode` in setup/teardown.

Run: `./gradlew test --tests "org.frc5010.examples.mechanisms.MechanismsFunctionalTest"`
(Windows local: `.\gradlew.bat`).

## Step 6 — Verify in the sim GUI (optional but recommended)

`./gradlew simulateJava`, enable, then watch the combined side-view overlay under
SmartDashboard → **RobotMechanisms** (set `settings.visualPosition` to where the
mechanism sits on the robot; pass `settings.mechanism2d` for a separate widget).
In `ExampleRobot`, holding X drives every example mechanism to a midpoint;
releasing returns them to start.

## Step 7 — Hand off to tuning

Point the user at `/tune-mechanism` for LQR weight tuning and real-robot kG/kV/kA
characterization. Defaults converge in sim; real robots need SysId.
