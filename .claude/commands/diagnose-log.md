# /diagnose-log — Analyze a .wpilog for problems and performance

Use this playbook to diagnose a simulation or robot run using its AdvantageKit log file.

---

## Step 0 — Check that a log exists

Logs are written to `logs/` during every `simulateJava` run. If the directory is empty, run the simulation first:

```powershell
.\gradlew.bat simulateJava        # or -PvisualTest, -PtestSim
```

Then list available logs:
```powershell
Get-ChildItem logs\*.wpilog | Sort-Object LastWriteTime | Select-Object Name, LastWriteTime
```

---

## Step 1 — Print the summary

```powershell
# Most recent log
.\gradlew.bat logSummary

# Specific file
.\gradlew.bat logSummary -PlogFile=logs\FRC_20260525_143022.wpilog
```

The output has four sections:

| Section | What to look at |
|---------|----------------|
| Header | Duration — unusually short runs may have crashed early |
| All Entries | Full list of logged signals and types; use to discover available data |
| Numeric Statistics | Min / max / count per `double` signal; spot outliers |
| Anomaly Flags | Explicit warnings for known bad patterns |

---

## Step 2 — Interpret anomaly flags

### Loop overrun (`EpochTimeMicros > 25 ms`)
The robot loop took longer than one 20 ms cycle. Causes:
- Heavy computation added to `periodic()` (pathfinding, vision processing)
- Too many subsystems registered with `CommandScheduler`
- GC pause (rare in WPILib — check if running in debug mode)

Fix: profile what runs in `robotPeriodic()`. Move expensive work to a background thread.

### Gyro disconnected
`RealOutputs/Drive/GyroConnected` went false at some point during the run.
- In sim: indicates a `GyroIOSim` vs `GyroIOSimPhysics` misconfiguration
- On hardware: CAN bus issue, wrong CAN ID, or device brownout

Fix: verify `gyroCanId` in `SwerveConstants`, check CAN bus health in Phoenix Tuner.

### High motor current (> 60 A drive)
A module drove at stall or near-stall current. Common causes:
- Collision (blocked wheel) — check physics body config
- Aggressive PID gains (steer oscillating)
- Incorrect `wheelRadius` — odometry thinks wheels are spinning faster than they are, commands more voltage

Fix: check `ModuleIOTalonFX` PID gains; verify `wheelRadius` matches actual worn wheel.

---

## Step 3 — Drill into a specific signal

The `logSummary` output gives you the exact entry name. To see the signal over time, use AdvantageScope:

1. Open AdvantageScope → **File → Open Log** → select your `.wpilog`
2. Drag the signal name from the left panel to a chart
3. Zoom to the time range of interest

For headless/agent use, all `double` min/max/count values are in the Numeric Statistics section — sufficient for most regression and performance checks.

---

## Step 4 — Replay (re-run code against the log)

If you changed the code and want to verify the fix against a previously captured failure:

```powershell
.\gradlew.bat replayWatch
```

Select the original log in the file picker. The robot code re-runs at full speed and writes `<original>_sim.wpilog`. Open both files in AdvantageScope and compare the same signal — discrepancies reveal where the code diverges from the recorded behavior.

Replay mode is also how you validate that a new algorithm improvement (e.g., better odometry, smoother heading control) produces better output on a known input.

---

## Step 5 — Performance analysis workflow

To compare before/after a code change:

1. Run simulation, save baseline: `.\gradlew.bat simulateJava -PvisualTest`
2. Note the log filename, e.g. `logs/FRC_20260525_140000.wpilog`
3. Make your code change
4. Run simulation again, get the new log
5. Run `logSummary` on both and diff the Numeric Statistics output

```powershell
.\gradlew.bat logSummary -PlogFile=logs\baseline.wpilog > before.txt
# ... make change ...
.\gradlew.bat logSummary -PlogFile=logs\after.wpilog > after.txt
Compare-Object (Get-Content before.txt) (Get-Content after.txt)
```

Key metrics to compare for swerve performance:
- `Module{0-3}DriveVelocityRadPerSec` max — did the robot reach commanded speed?
- `Module{0-3}DriveCurrentAmps` max — did efficiency improve?
- `Drive/Pose` range — did the robot travel the expected distance?

---

## Common patterns

| Observation | Likely cause |
|-------------|-------------|
| Pose X/Y never moves from 0 | DriverStation not enabled; check `DriverStation.isDisabled()` gate in `periodic()` |
| Pose moves but rotation wrong | Gyro sign issue — check `yaw` field sign convention in `GyroIOPigeon2` |
| Strafe min/max < 0.05 m after 1 s at 1 m/s | Modules didn't rotate to correct angle — steer PID may be too slow |
| Current spikes every ~0.5 s | Steer PID oscillating — reduce D gain |
| Log duration = 0.02 s | Robot crashed on init — check startup logs for exception |
