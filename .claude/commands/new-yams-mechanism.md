# /new-yams-mechanism — Add a YAMS mechanism subsystem

Agent playbook for adding an elevator, arm, pivot/turret, flywheel/shooter,
double-jointed arm, or differential wrist to a robot using the YAMS wrappers in
`org.frc5010.common.mechanisms`. Read [docs/mechanisms.md](../../docs/mechanisms.md)
first — especially the Gotchas section.

## Step 1 — Pick the mechanism type and control style

| Robot part | Wrapper | Default controller |
|---|---|---|
| Elevator, climber (linear travel) | `YamsElevator` | ELEVATOR LQR (meters) |
| Arm with gravity (shoulder, intake pivot) | `YamsArm` | ARM LQR + kG·cos(θ) |
| Turret, hood, gravity-free wrist | `YamsPivot` | ARM LQR, no gravity FF |
| Shooter wheel, roller (velocity) | `YamsFlywheel` | FLYWHEEL LQR |
| Two-jointed arm (shoulder+elbow) | `YamsDoubleJointedArm` | profiled PID per joint (only style) |
| Tilt+twist differential wrist | `YamsDifferentialMechanism` | profiled PID per motor (only style) |

The four LQR wrappers also support `s.controlStyle = ControlStyle.PROFILED_PID`
(trapezoid profile + kP/kI/kD + kS/kV/kG feedforward, onboard MotionMagic on TalonFX).
Pick PROFILED_PID when the team prefers hand-tunable gains or the mass/MOI model is
uncertain; pick LQR when the physical parameters are known. Profiled examples:
`ExampleProfiledElevator/Arm/Turret/Shooter` (CAN 31–34).

## Step 2 — Gather the robot-specific numbers

Required before writing code (ask the user if unknown):
- Motor controller vendor + CAN ID(s). TalonFX (Kraken X60) is the project default.
- Gear reduction stages rotor→mechanism (e.g. `{4, 3}` = 12:1).
- Elevator: drum/sprocket circumference, carriage mass, min/max/start height.
- Arm: length, mass, hard limits, starting angle.
- Pivot: MOI of the rotating assembly, limits, starting angle.
- Flywheel: wheel diameter + mass (or MOI).
- **Profile cruise velocity must be below free speed ÷ gearing × circumference** —
  compute it and check, don't trust the user's wish speed.

## Step 3 — Create the subsystem in TEAM code (`frc.robot.mechanisms`)

Copy the matching `Example*` class from `src/main/java/frc/robot/mechanisms/` and
replace the constants. Keep the pattern: a thin class extending the common wrapper
whose only content is a `settings()` method. **Never put control logic in team code**
— if something is missing from the wrapper, extend the common class in
`org.frc5010.common.mechanisms` instead.

## Step 4 — Wire commands

The wrapper exposes ready-made commands: `goToHeight`/`goToAngle`/`goToSpeed`
(never finish — bind to buttons or sequence with `.until(isAt...(...))`),
`setDutyCycle` for manual jog, `sysId()` for characterization.

## Step 5 — Add a functional test

Add a test to `src/test/java/frc/robot/mechanisms/YamsMechanismsFunctionalTest.java`
(or a new class copying its `runScheduledFor` pump — do NOT use plain
`SimTestBase.runFor`, the synchronous `stepTiming` deadlocks against the YAMS
Notifier). Schedule the command, run ~4 sim-seconds, then use `assertConverges`.
Always `close()` the subsystem in a `finally` block to free CAN IDs.

Run: `./gradlew test --tests "frc.robot.mechanisms.YamsMechanismsFunctionalTest"`
(Windows local: `.\gradlew.bat`).

## Step 6 — Verify in the sim GUI (optional but recommended)

`./gradlew simulateJava`, then watch SmartDashboard → `<name>/mechanism`
(Mechanism2d) while driving the mechanism from the YAMS telemetry entries.

## Step 7 — Hand off to tuning

Point the user at `/tune-mechanism` for LQR weight tuning and real-robot kG
characterization. Defaults converge in sim; real robots need sysId.
