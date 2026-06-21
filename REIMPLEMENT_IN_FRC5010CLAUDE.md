# Re-implementation Prompt — Port FRC Team 5010 "Rebuilt2026" robot into FRC5010Claude

> **How to use this file.** Hand the whole document to Claude Code running in
> `C:\workspace\FRC5010Claude`. It is a behavioral + configuration specification of the
> 2026 competition robot ("Rebuilt2026"). Re-implement the same *functionality* using
> FRC5010Claude's existing primitives (swerve drive, `org.frc5010.common.mechanisms.*`
> TalonFX mechanisms, `Vision`, `LedStripSegments`, AdvantageKit IO pattern, the
> `RobotProfile` / `SwerveRobotContainer` structure). Do **not** copy the Rebuilt2026
> config-driven JSON/YAMS stack — translate each mechanism to the equivalent
> FRC5010Claude class. Build incrementally, one subsystem at a time, with a Layer 2/3 sim
> test per subsystem (see the FRC5010Claude test pyramid).
>
> **The source robot is checked out at `C:\workspace\Rebuilt2026` — read it directly.**
> This document is the map (behavior, parameters, what-maps-to-what); when you need exact
> values (CAN IDs, swerve module offsets, full gain sets, table rows), open the source
> rather than relying on this summary. Season code is under
> `src/main/java/frc/robot/rebuilt/**` and hardware configs under
> `src/main/deploy/rebuilt_robot/**`.
>
> **Out of scope: Climb.** Skip the climb subsystem entirely — it is disabled in the source
> robot and not needed in the port.

---

## 0. Source architecture (what you are porting FROM)

Rebuilt2026 is config-driven: hardware is described in `src/main/deploy/rebuilt_robot/**.json`
and instantiated by parsers into a `subsystems` map. Mechanisms use the **YAMS** library
(`yams_arm`, `yams_shooter`/flywheel, `yams_turret`/pivot, `yams_elevator`). Every
hardware-touching subsystem follows the **AdvantageKit IO split** (`Subsystem` +
`SubsystemIO` `@AutoLog` + `IOReal`/`IOSim`). Subsystem behavior is driven by **state
machines** (`org.frc5010.common.arch.StateMachine`) whose transitions are gated on a
`stateRequested` vs `stateCurrent` pair stored in the `@AutoLog` inputs.

**FRC5010Claude equivalent to target:**

| Rebuilt2026 (YAMS, config-driven)          | FRC5010Claude class to use                         |
|--------------------------------------------|----------------------------------------------------|
| `AKIT_SWERVE_DRIVE` from `robot.json`      | `SwerveFactory.build()` + `SwerveConstants`        |
| `yams_arm` (hood, hopper)                  | `org.frc5010.common.mechanisms.Arm`                |
| `yams_turret` (turret)                     | `org.frc5010.common.mechanisms.Pivot` (YAW_PLANE) + custom controller (see §5) |
| `yams_shooter` (flywheel, spintakes)       | `org.frc5010.common.mechanisms.Flywheel`           |
| `percent_motor` (spindexer, transfer)      | simple duty-cycle TalonFX (no mechanism class)     |
| `LEDStrip` / `ConfigConstants.ALL_LEDS`    | `LedStripSegments` + a `DemoLeds`-style mapper      |
| 4× cameras (`cameras/*.json`)              | `Vision` + `CameraConfig` (PhotonVision)           |
| `StateMachine` (`org.frc5010.common.arch`) | port the class, or model states with WPILib `Trigger`s |

Each mechanism in FRC5010Claude already supports REAL / SIM / REPLAY via `RobotMode.get()`
— set `RobotMode` before constructing any mechanism.

---

## 1. Game model (2026 season — internal name "Rebuilt")

- **Game piece:** "Fuel" (ball). `robot.json` → `gamePieceA: "Fuel"`.
- **Scoring target:** the **Hub** — a tall central goal. Top-center aim point is
  `FieldConstants.Hub.topCenterPoint` (x = tag-26 X + hubWidth/2, y = fieldWidth/2,
  z = 72 in). Robots shoot Fuel into it with a turreted shooter.
- **Tower:** a field structure near the alliance wall (rungs at 27/45/63 in). Relevant here
  only as a **preset shot location** (`Tower.face`) — the climb mechanism is out of scope.
- **AndyMark field**, AprilTag layout from `org.frc5010.common.vision.AprilTags`. Key
  tags: 26 = near hub face (alliance), 4 = opposite hub, 31 = alliance tower, 18/20/21 =
  hub faces, 29 = outpost. Field has left/right **Bumps** and **Trenches** (low-ceiling
  passages — the turret must duck under them).
- **Hub "shift" schedule (`HubTracker`)** — the hub is only *active* (scorable) for one
  alliance during alternating 25-second windows, keyed off who won auto (FMS game-specific
  message first char `R`/`B`). Match-time model: auto counts 0–20 s, teleop 0–160 s.
  Shifts: AUTO(0–20, both), TRANSITION(20–30, both), SHIFT_1(30–55, auto-loser),
  SHIFT_2(55–80, auto-winner), SHIFT_3(80–105, auto-loser), SHIFT_4(105–130, auto-winner),
  ENDGAME(130–160, both). Port `HubTracker` and `FieldConstants` verbatim (pure geometry +
  time math, no hardware) — they are the field/strategy model the launcher and driver
  display read from.

---

## 2. Robot-wide configuration

**Drivetrain (`robot.json`):** swerve, track width 22 in, wheelbase 22 in, wheel diameter
0.1002156588 m, physical max speed 5.93 m/s, drive gear ratio 1.0. Field sim loaded.
→ Fill these into a `SwerveConstants` builder. Use the existing swerve stack as-is.

**Brownout:** `RobotController.setBrownoutVoltage(4.6 V)`.

**Mode select:** REAL when `RobotBase.isReal()`, else SIM, else REPLAY when launched with
`-Dlog=<path>` / `AKIT_LOG_PATH` (this replay wiring already exists in FRC5010Claude).

**CAN map (CANivore bus "canivore" for the turret; rest on rio bus):**

| CAN | Mechanism            | Motor      | Notes                                  |
|-----|----------------------|------------|----------------------------------------|
| 13  | spintake_outer       | KrakenX60  | inverted                               |
| 14  | hopper follower      | KrakenX44  | inverted                               |
| 15  | hopper (arm)         | KrakenX44  | inverted, follower 14                  |
| 16  | flywheel             | KrakenX60  | inverted, follower 17                  |
| 17  | flywheel follower    | KrakenX60  | inverted                               |
| 18  | turret (pivot)       | KrakenX44  | inverted, **CANivore**                 |
| 19  | hood (arm)           | KrakenX44  | current limit 60 A                     |
| —   | spintake_inner       | (see `intake/spintake_inner.json`) |                    |
| —   | spindexer            | percent/duty TalonFX |                              |
| —   | transfer_flywheel    | (see `indexer/transfer_flywheel.json`) |                  |

> Exact gains/ratios per mechanism are in §3–§6. Other CAN IDs and the swerve module IDs
> live in `src/main/deploy/rebuilt_robot/` of the source repo — pull them as needed.

---

## 3. Subsystem: Intake  (`frc.robot.rebuilt.subsystems.intake`)

**Hardware:** two spintake roller groups (`spintake_inner`, `spintake_outer`, velocity
flywheels) + a **hopper arm** that deploys/retracts the intake (KrakenX44 ×2, CAN 15/14,
24:1 gear, PROFILED control).

**Hopper arm config (`intake/hopper.json`):** PROFILED MotionMagic, real gains
P=40 D=5, FF s=0.599 v=0.01006 g=0.65; sim P=10 g=0.04055; maxVel 1080 °/s, maxAccel
5000 °/s². Hard limits 0°–125°, soft 0°–120°, starts at 125° (retracted), current limit
60 A. **0° = deployed (hard stop on floor), 120° = retracted.**

**Hopper homing/zeroing logic (important, port carefully):**
- The hopper has no absolute encoder. It homes against the **deployed hard stop** (0°).
- "Hard stop detected" = low movement velocity AND stall current (threshold 80 A),
  debounced 0.3 s (`HOPPER_STALL_TIME`).
- First deploy: drive open-loop down at duty `HOPPER_FIRST_DEPLOY_DUTY = -0.2` until hard
  stop, then `zeroHopper()` (set encoder to 0°) and mark `hopperZeroed`.
- Subsequent deploys (zeroed): PID to 0°, then nudge at `-0.35` (`HOPPER_DEPLOY_NUDGE_DUTY`)
  until hard stop, 1.5 s timeout.
- **Auto-rezero** runs every periodic: if zeroed + on the deploy side + hard stop detected +
  angle between tolerance(3°) and `HOPPER_DEPLOY_STOP_REZERO_MAX_ANGLE`(20°), OR if angle
  drops below `HOPPER_AUTO_REZERO_THRESHOLD`(-1°), reset encoder to 0°.

**Spintake speeds (`Constants.Intake`):** INTAKE_IN 1.0, INTAKE_INNER_IN 0.3, INTAKE_AUTO
1.0, INTAKE_CHURN -0.25, deadzone 0.25, max in/out ±0.9.

**State machine (`IntakeState`):** UNKNOWN, RETRACTED, RETRACTING, DEPLOYING, INTAKING,
DEPLOYED, ANGLED. Driven by `stateRequested`/`stateCurrent` + WPILib `Trigger`s:
- request INTAKING → `deployingCommand()` until current==INTAKING, then `intakingCommand`
  (runs hopper-home logic + spintakes at trigger-controlled speed; forces INTAKE_AUTO in
  autonomous).
- request RETRACTING → stop spintakes, PID hopper to 120°, until moving → RETRACTED.
- ANGLED → PID hopper to 45° then run outer spintake at 0.5×, inner at churn (clears jams).
- `isNearTrench()` true while DEPLOYING near a trench opening — used to duck the turret.

---

## 4. Subsystem: Indexer  (`frc.robot.rebuilt.subsystems.Indexer`)

**Hardware:** `spindexer` (percent/duty motor, agitates Fuel) + `transfer_flywheel`
(velocity, feeds Fuel up to the launcher).

**Speeds (`Constants.Indexer`):** SPINDEXER_SPEED 0.90, TRANSFER_SPEED 1.0,
TRANSFER_CHURN 0.25.

**State machine (`IndexerState`):** IDLE, CHURN, HARD_CHURN, FORCE, FEED.
- **FEED** — spindexer 0.90 + transfer 1.0 (shoot). LEDs rainbow.
- **FORCE** — same speeds, used to force-feed regardless of launcher readiness.
- **CHURN** — gentle reverse spindexer (-0.1) + transfer 0.25 for 1 s then stop (keeps Fuel
  from jamming while spinning up). Only physically churns once
  `LauncherCommands.isFlywheelReadyForChurn()` is true (flywheel at/above goal — gated by
  `requireFlywheelAtGoalForChurn`, default true).
- **HARD_CHURN** — stronger reverse spindexer (-0.5) + transfer 0.25 (driver-held un-jam).
- **IDLE** — all stop, LEDs rainbow.

**Launcher↔Indexer coupling (wired in `LauncherCommands.configureStateMachine`):**
- launcher in PREP **and** `isAtGoal()` → indexer FEED (fire).
- launcher in PREP **and** not at goal → indexer CHURN (keep agitating while spinning up).
- otherwise → indexer IDLE.

---

## 5. Subsystem: Launcher  (`frc.robot.rebuilt.subsystems.Launcher`) — the hard one

Three mechanisms: **flywheel** (dual KrakenX60, CAN 16/17, 18:1, velocity), **hood** (Arm,
KrakenX44 CAN 19, gear 1015:33 ≈ 30.76:1, PROFILED), **turret** (Pivot, KrakenX44 CAN 18,
30:1, on CANivore).

### 5.1 Flywheel (`launcher/flywheel.json`)
Velocity control, real P=5 FF s=0.0669 v=2.1 a=0.388; sim P=2 v=2.1962. Soft limits
0–5000 RPM, wheel radius 1.975 in, mass 5.1 lb. → `Flywheel` mechanism, PROFILED_PID
(needs kV ≈ 12 ÷ free-speed-rot-s).

### 5.2 Hood (`launcher/hood.json`)
PROFILED MotionMagic, real P=1100 D=60 FF a=0.0972 g=4.357; sim P=9. maxVel 1080 °/s,
maxAccel 5000 °/s². **Hard + soft limits 12.723°–45.723°, starts 12.723°.** length 9.466 in.
- The mechanical hood has a "legacy" angle frame offset by
  `HOOD_CALIBRATION_OFFSET = 12.723 − 30 = −17.277°`. The shot tables are authored in
  *legacy* degrees; `Constants.Launcher.offsetLegacyHoodAngleDegrees()` converts to real.
  **Port this offset exactly** or every shot table value is wrong.
- Hood zeroing: drive down (`runHoodDown`) 0.75 s, stop, `resetHoodAngle(12.723°)`.
- Stall detection at 20 A (`HOOD_STALL_CURRENT_THRESHOLD`).
- Tolerances: hood ±3.5°, flywheel ±50 RPM, turret ±5°.

### 5.3 Turret (`launcher/turret.json`) — custom `SmartTurretController`
30:1, real FB P=1770 D=1477 FF s=12.2 v=3.06 a=1.948; sim P=8 D=8. maxVel 1080 °/s
(model allows up to ~360 °/s real), hard ±160°, soft ±150°. Robot→turret offset
(−4.856, 4.863, 14.466) in. **The turret is a 2-state torque-current-FOC controller you
must port** (FRC5010Claude's LQR/MotionMagic mechanism classes don't cover this) — see
`SmartTurretController.java`:
- **SEEKING** (large error): `MotionMagicExpoTorqueCurrentFOC` — smooth long travel.
- **TRACKING** (small error): `PositionTorqueCurrentFOC` + explicit velocity & acceleration
  feedforward — tracks a moving target.
- Hysteresis: SEEK→TRACK when |err| < threshold; TRACK→SEEK when |err| > threshold+buffer.
- Runs at **200 Hz via a `Notifier`** (`step(0.005)`); `setTarget()` is called from the
  20 ms loop. Thread-safe via `AtomicReference<TurretTarget>` (an immutable record of
  position/velocity/accel).
- Zeroing: `zeroTurretCommand()` resets turret+hood to zero, only when disabled and not
  (already-enabled && FMS). Plays a tone + rainbow LED burst. Bound to operator Start.

### 5.4 ShotCalculator (`ShotCalculator.java`) — moving-shot solver (port faithfully)
Singleton. Holds **interpolating lookup tables** keyed on distance-to-target (meters):
hood angle (legacy°), flywheel speed, time-of-flight. Default tables are in
`createDefaultTables()` (18 hood/speed rows 1.42–13.02 m, 12 TOF rows; min 0.7 m, max
100 m, phase delay 0.03 s). `flywheelMultiplier` default **1.05**, operator-adjustable
±0.01 via POV/bumpers.

Per shot it:
1. snapshots robot pose, **field velocity, field acceleration** from the drivetrain;
2. extrapolates the pose forward by `phaseDelay` using **linear field-frame** extrapolation
   (not `Pose2d.exp` — the drive already discretizes for curvature);
3. delegates to `TurretControlPhysics.solve(...)` which iterates a virtual-target solution
   accounting for time-of-flight and turret travel time (velocity-aware settling-time
   function from turret maxVel/maxAccel), respecting turret ±165° limits + 10° FF padding;
4. returns `ShootingParameters{isValid, turretAngle, turretVelocity, hoodAngle, hoodVelocity,
   flywheelSpeed, distanceToVirtualTarget, solution}`;
5. logs `ShotCalculator/*` outputs and pushes Field2d objects "Target", "Lookahead",
   "VirtualTarget", "Turret" for AdvantageScope/Glass.
Also supports a ballistic-table generator (`createBallisticTables`/`solveBallistic`) and
NORMAL vs SHUTTLE shot profiles. `getParameters` caches `latestParameters` per cycle.

### 5.5 Launcher state machine (`LauncherState` in `LauncherCommands`)
States: IDLE, LOW_SPEED, PREP, HAMMERTIME, AUTO_HAMMERTIME, ESCAPE_HAMMERTIME, PRESET.
Initial = HAMMERTIME.
- **HAMMERTIME** — turret forced to 0°, hood LOW (31° legacy), flywheel idle (1 RPS). This
  is the "stow under trench / intake interference" safe pose. Toggle state.
- **LOW_SPEED** — track target turret+flywheel but hold hood at its preset low angle (ready
  to aim, low spin). LEDs solid green.
- **PREP** — full `trackTargetCommand()`: hood + turret (with FF) + flywheel all from
  ShotCalculator. When at goal, indexer feeds. LEDs rainbow.
- **PRESET** — fixed precomputed hood/turret/flywheel for a named field spot. Presets:
  tower, left corner, right corner, turret-forward (operator A/B/X/Y while-held).
- **AUTO_HAMMERTIME / ESCAPE_HAMMERTIME** — auto-duck when entering a trench, then restore
  the pre-trench state (`isNearTrench()` drives this; currently commented in bindings but
  the states/transitions exist).
- Intake coupling: when intake is RETRACTING/RETRACTED → force HAMMERTIME (hopper arm blocks
  turret); when intake deploys clear of the turret → return to LOW_SPEED.

---

## 6. Subsystem: Climb — **out of scope, skip**
The climb elevator is disabled in the source robot (`// climb = new Climb();` in
`Rebuilt.java`) and is not part of this port. Ignore the `Climb*` classes, `ClimbCommands`,
the `climb*` named commands, and the climb-related controller bindings.

---

## 7. Driver display & LEDs
- **`HubStatus`** subsystem — logs hub shift state for the dashboard (rumble on shift-change
  is stubbed/commented; the `@AutoLog` inputs track `timeRemainingInCurrentShift`).
- **LEDs** via `LEDStrip.changeSegmentPattern(ConfigConstants.ALL_LEDS, pattern)`:
  disabled → green if turret zeroed else alliance color; turret-zero burst → rainbow;
  launcher LOW → solid green, PREP → rainbow; indexer FEED/IDLE → rainbow.
  → reproduce with `LedStripSegments` + a `DemoLeds`-style state mapper.
- **OrchestraManager** — plays robot-音 tones via TalonFX Orchestra (zero-turret tone
  261.63 Hz; music loadable). Optional/nice-to-have.

---

## 8. Controls (Xbox driver port 0, operator port 1)

**Driver:**
- Left Y / Left X / Right X → swerve drive (configured axes, with deadbands).
- **Y** → toggle field-oriented drive.
- **B** (hold) → launcher PREP; release → LOW_SPEED.
- **A** → LOW_SPEED.
- **Right/Left Trigger** (limited to 0.9, deadband 0.25) → intake (right−left = signed speed).
- **Right Bumper** → intake RETRACTING.
- **Left Bumper** (hold) → indexer HARD_CHURN; release → CHURN.
- **X** → hopper-down + intake.
- **Start** → hopper retracted; **Back** → hopper deployed.
- *(POV-Up enables climb in the source — skip, climb out of scope.)*

**Operator:**
- **A/B/X/Y** (hold) → tower / right-corner / left-corner / turret-forward PRESET; release → LOW_SPEED.
- **Left Bumper** (hold) → FORCE feed if OK-to-fire else CHURN; release → CHURN.
- **Right Bumper** → intake ANGLED (hold) / INTAKING (release).
- **POV Up/Down/Left/Right** → flywheel multiplier ±0.01.
- **Back** → zero-hood sequence.
- **Start** → zero turret (disabled only).
- **Down POV** → hopper-down.

---

## 9. Autonomous
- PathPlanner-based. `NamedCommandsReg` registers (climb entries omitted):
  `launcherPrep/Preset/Low/Idle`, `intakeIntake/Retracted/Retracting`,
  `indexerChurn/Idle/Feed`, `iForcePreset`, `hubPreset` (=left-corner preset),
  `towerPreset`, `towerForwardPreset`, `WaitUntilIntaking`.
- Auto chooser also offers "Do Nothing", a "Shoot Preload Only" sequence, and a suite of
  **characterization/tuning routines** (hood/turret/flywheel SysId, turret quasistatic /
  dynamic / kS-map / tracking-tune / seeking-tune, shot-lookup-table tuning). Port the
  characterization commands as optional chooser entries.
→ FRC5010Claude uses BLine for autos; either wire PathPlanner or express these as BLine
routines that call the same named command building blocks.

---

## 10. Logging signals (AdvantageScope / replay)
Each subsystem logs its `@AutoLog` inputs under its name: `Launcher/*`, `Intake/*`,
`Indexer/*`, `HubStatus/*`, plus `ShotCalculator/*` outputs and the Field2d
objects in §5.4. Keep `@AutoLog` fields primitive (`double`, never `Measure<>`) — extract
`.in(unit)` before assigning. The Launcher inputs include the full aim solution
(calculated/desired/actual hood-turret-flywheel, errors, at-goal flags, turret FF vel/accel,
distance-to-virtual-target, state enums). This is what makes the robot replayable — preserve
the signal set so `/validate-replay` and AdvantageScope work.

---

## 11. Suggested build order (each with a sim test before moving on)
1. **Swerve** — `SwerveConstants` from §2, confirm it drives in sim.
2. **Intake** — spintakes + hopper arm + homing/auto-rezero state machine.
3. **Indexer** — spindexer + transfer + state machine.
4. **Launcher flywheel + hood** — mechanisms + tolerances + hood legacy offset.
5. **ShotCalculator + TurretControlPhysics** — port the solver with a unit test against the
   default tables (deterministic, no hardware).
6. **Turret + SmartTurretController** — 200 Hz Notifier, SEEKING/TRACKING.
7. **Launcher state machine** + indexer coupling + intake interference rules.
8. **HubStatus/LEDs/Orchestra**, **Vision** (4 cameras).
9. **Autos** + named commands + characterization chooser entries.

> When in doubt about a parameter, the authoritative values are in
> `C:\workspace\Rebuilt2026\src\main\deploy\rebuilt_robot\**` and
> `frc.robot.rebuilt.Constants`. This document captures the behavior and the numbers that
> matter; pull exact CAN IDs / module offsets from the deploy tree as you wire each piece.
