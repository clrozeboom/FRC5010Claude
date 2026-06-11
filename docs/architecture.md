# Architecture

---

## Layer diagram

```
SwerveConstants (immutable record, Builder — all fields typed WPILib units)
        │
        ▼
SwerveFactory.build()               ─── SIM: IronMaple physics (SwerveDriveSimulation)
SwerveFactory.buildWithoutPhysics() ─── SIM: WPILib DCMotorSim (lighter; for unit tests)
        │
        ▼
AkitSwerveDrive (SubsystemBase — AdvantageKit IO abstraction)
 ├── GyroIO
 │    ├── GyroIOPigeon2        (REAL)
 │    ├── GyroIONavX           (REAL)
 │    ├── GyroIOSimPhysics     (SIM via build() — reads IronMaple GyroSimulation)
 │    └── GyroIOSim            (SIM via buildWithoutPhysics() — kinematics integration)
 └── ModuleIO [× 4]
      ├── ModuleIOTalonFXReal  (REAL — needs CTRE TunerX SwerveModuleConstants)
      ├── ModuleIOSparkTalon   (REAL — NEO drive + Falcon steer)
      ├── ModuleIOSimPhysics   (SIM via build() — IronMaple GenericMotorController)
      └── ModuleIOSim          (SIM via buildWithoutPhysics() — DCMotorSim)

RobotProfile (abstract)
 ├── SimRobotProfile           (library CI / dev — no real CAN IDs)
 └── ExampleRobotProfile          (frc.robot — team's robot; branches on RobotBase.isReal())

SwerveRobotContainer (abstract — keyboard drive, alliance pose, auto dispatch)
 ├── ConfigurableController    (wraps GenericHID port → JoystickAxis + Trigger)
 │    ├── XboxConfigurableController  (named accessors: leftY(), rightX(), a(), leftBumper(), …)
 │    │    └── WebXboxController      (injects web button suppliers; used when -PwebUI is set)
 │    └── JoystickAxis   (chainable: deadzone → power → scale → limit → negate)
 │         └── DriveVector    (two JoystickAxis → Translation2d, magnitude transforms, unitCircle())
 ├── WebControl (optional singleton — HTTP server port 5800, web button injection; -PwebUI only)
 └── frc.robot.ExampleRobot (concrete — extends SwerveRobotContainer, owns DemoIntake)
      └── frc.robot.RobotContainer (thin shell — delegates getAutonomousCommand() and resetToAllianceStart())

SimRobotState (abstract SubsystemBase — IntakeSimulation lifecycle, web UI state binding)
 └── frc.robot.DemoIntake (2026 Rebuilt: Fuel piece intake, ballistic firing, hub scoring)
```

---

## Factory mode selection

`SwerveFactory.build()` and `buildWithoutPhysics()` inspect `RobotMode` to select the right IO stack:

| `RobotMode` | `build()` | `buildWithoutPhysics()` |
|-------------|-----------|------------------------|
| `SIM` | `ModuleIOSimPhysics` + `GyroIOSimPhysics` | `ModuleIOSim` + `GyroIOSim` |
| `REAL` with `SIM` module/gyro types | `ModuleIOSimPhysics` + `GyroIOSimPhysics` | same |
| `REAL` with `TALON_FX` or `SPARK_TALON` | throws `UnsupportedOperationException` | throws |
| `REPLAY` | no-op `ModuleIO` + `GyroIO` | same |

The throw-by-design for hardware types in REAL mode is intentional — the factory cannot construct motor configs without full TunerX gear-ratio and gain constants. Teams wire those directly in `ExampleRobotProfile.createDrive()`.

---

## Critical distinction — `GyroIOSim` vs `GyroIOSimPhysics`

`AkitSwerveDrive.periodic()` contains:

```java
if (gyroIO instanceof GyroIOSim simGyro) {
    // kinematics-fallback branch: integrate wheel velocities → heading
    simGyro.updateAngularVelocity(speeds.omegaRadiansPerSecond);
}
```

- **`buildWithoutPhysics()`** injects `GyroIOSim` → this branch fires. The gyro heading is derived from wheel kinematics, not a physics engine.
- **`build()`** injects `GyroIOSimPhysics` (a separate class) → this branch does **not** fire. The heading comes from the IronMaple `GyroSimulation`, which integrates dyn4j angular velocity.

This distinction is the key reason `buildWithoutPhysics()` exists: it provides a deterministic, physics-free path suitable for fast unit tests.

---

## Per-cycle call order — Layer 3 tests (IronMaple)

```
drive.runVelocity(speeds)     1. queue voltage commands to physics motor controllers
drive.simulationPeriodic()    2. advance dyn4j world: 5 sub-ticks × 4 ms = 20 ms
                                 ├─ each sub-tick: integrate forces, update positions
                                 └─ fill module position caches (5 entries each)
drive.periodic()              3. read updated caches → SwerveDrivePoseEstimator
                                 └─ publish Field2d, AdvantageKit logs
stepOneCycle()                4. advance FPGA clock 20 ms
```

**Wrong order = stale data.** `SwerveModuleSimulation` pre-fills its caches with 5 copies of the initial zero position at construction. Without `simulationPeriodic()`, those 5 zeros are re-read every cycle — pose stays at origin regardless of commanded velocity.

Layer 2 tests (`buildWithoutPhysics`) skip step 2; `ModuleIOSim.updateInputs()` calls `driveSim.update(0.02)` internally.

---

## Per-cycle call order — Layer 4 robot program

`CommandScheduler.run()` (called from `Robot.robotPeriodic()`) handles everything:

```
robotPeriodic()
  └─ CommandScheduler.run()
       ├─ drive.periodic()           // read sensors → odometry → Field2d
       ├─ drive.simulationPeriodic() // advance IronMaple physics (sim only)
       └─ command.execute()          // runVelocityFieldRelative() or visual-test step
```

There is a **1-cycle lag** between commanding velocity and seeing pose displacement — this is normal for a real-time control loop. Do not add a separate `drive.simulationPeriodic()` call in `Robot.simulationPeriodic()`; the scheduler already calls it and a double-call would advance the physics engine twice per loop.

---

## High-frequency odometry (Phoenix Pro / CANivore)

For TalonFX modules on a CANivore bus, `TalonFXOdometryThread` runs at `odometryFrequency` (default 100 Hz, up to 250 Hz) and queues timestamped yaw and position readings between main-loop `periodic()` calls. `SwerveDrivePoseEstimator.updateWithTime()` uses these intermediate samples to reduce dead-reckoning error.

The thread wakes via `BaseStatusSignal.waitForAll(2.0 / odometryFrequency, signals)` on CANFD, or `Thread.sleep(1000 / odometryFrequency)` on standard CAN.

For SparkMAX modules, `SparkOdometryThread` provides the same queue with a periodic-frame-callback approach.

---

## AdvantageKit IO abstraction

Every hardware type (gyro, module) implements a thin interface (`GyroIO`, `ModuleIO`) with a single `updateInputs(Inputs)` method. The `@AutoLog`-generated `Inputs` inner class holds primitive `double` fields that AdvantageKit serialises into the `.wpilog`.

**Do not convert `@AutoLog` fields to `Measure<>`** — the annotation processor generates code that expects `double`. Extract the raw value with `.in(unit)` before assigning to an inputs struct.

In REPLAY mode, `Logger.processInputs()` replays the logged values into the same inputs struct without calling the real `updateInputs()` at all, enabling deterministic log replay with zero hardware access.
