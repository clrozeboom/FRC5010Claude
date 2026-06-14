# Simulation

The library uses **IronMaple** (a dyn4j-based physics engine bundled with YAGSL) for full 2D rigid-body simulation. The same `SwerveConstants` you configure for your real robot drive the physics parameters — robot mass, bumper geometry, and wheel placement all feed directly into the simulation model.

---

## Three simulation scenarios

### 1. Lightweight test sim (`-PtestSim`)

Uses `SimRobotProfile` — small square robot, no real CAN IDs. Intended for library development, CI, and automated test agents.

```powershell
.\gradlew.bat simulateJava -PtestSim   # Windows
./gradlew simulateJava -PtestSim        # Linux / macOS / Codespaces
```

### 2. Real-robot-in-sim (default)

Uses `ExampleRobotProfile` with your robot's actual `SwerveConstants`. IronMaple runs the physics using your real robot's mass, bumper footprint, and wheel geometry, so the simulation behaves like your actual robot. This is the default when you click **Simulate Robot Code** in VS Code.

```powershell
.\gradlew.bat simulateJava   # Windows — default
./gradlew simulateJava        # macOS / Linux / Codespaces
```

The drivetrain publishes a **`RobotMechanisms3D` Mechanism2d** to SmartDashboard
via the isometric canvas (`AkitSwerveDrive` → `MechanismVisuals3d` → `MechanismIsoCanvas`):
a 30° isometric projection of the chassis box, swerve wheels (live azimuth + speed),
gyro-heading compass, and all active mechanism segments, so Glass / Shuffleboard /
AdvantageScope show the full robot state without the web UI.

### 3. Visual auto-test (`-PvisualTest`)

Same as scenario 2, but `getAutonomousCommand()` returns the `SwerveVisualTest` sequence: the robot drives forward, strafes, rotates, verifies alliance-direction, and approaches the field boundary. Useful for a quick sanity check after changing constants.

```powershell
.\gradlew.bat simulateJava -PvisualTest   # Windows
./gradlew simulateJava -PvisualTest        # macOS / Linux / Codespaces
```

Once Glass opens, find the **Driver Station** panel, set mode to **Autonomous**, and click **Enable** to start the sequence. In Codespaces, use `xvfb-run` and connect AdvantageScope instead — see [Live connection — cloud environment](#live-connection--github-codespaces-or-cloud-environment) below.

### 4. Browser web UI (`-PwebUI`)

Opt-in browser-based UI on `http://localhost:5800` — field view, virtual gamepad, a Driver Station panel with alliance/enable controls, an **Auto/Teleop mode** toggle, an **auto-routine selector**, the `DemoIntake` demo, a **Mechanisms 3D** isometric panel (bottom-right of the field; live wireframe of the chassis, the swerve wheels (steered live, line length growing with drive speed), a cyan gyro-heading compass on the floor (`gyroRad` from `/api/state`), and every mechanism at its `visualPose3d` mount — drag to orbit, click the title to collapse it (which stops its poll/draw, handy on narrow screens); backed by `/api/mechanisms3d`; see [docs/mechanisms.md](mechanisms.md)), a held/scored counter panel in the top-right corner, and a **live LED strip display** under the field (mirrors the robot's `LedStripSegments` colours via the `"leds"` array in `/api/state`; hidden when the robot has no strip — see [docs/leds.md](leds.md)). `-PwebUI` skips `wpi.sim.addGui()` / `addDriverstation()`, so **no Glass window opens**; the browser is the only UI.

```powershell
.\gradlew.bat simulateJava -PwebUI         # Windows
./gradlew simulateJava -PwebUI              # macOS / Linux / Codespaces
```

`WebDriveController` drives `DriverStationSim` programmatically (setEnabled + setDsAttached + alliance + autonomous), so the Glass DS widget isn't needed. Plain `simulateJava` is the reverse — Glass only, no HTTP server on port 5800, no demo intake instance.

**Running an autonomous routine from the web UI:** pick a routine from the **Auto Routine** dropdown (mirrors the Glass `SendableChooser` registered in `ExampleRobot`), click **Auto** under **Mode**, then click **Enable**. Like a real Driver Station, the mode applies on the next enable — to switch back, **Disable**, click **Teleop**, then **Enable**. The selector and mode buttons drive `/api/control` (`mode`, `auto` fields) and `/api/autos`; the chosen routine becomes what `ExampleRobot.getAutonomousCommand()` returns when `autonomousInit()` fires.

#### Keyboard controls

The on-screen sticks and buttons can also be driven from the keyboard (each button shows its key hint). You must **Enable** (Space) before the robot moves in teleop.

| Key(s) | Action |
|--------|--------|
| `W` / `S` (or `↑` / `↓`) | Drive forward / backward |
| `A` / `D` | Strafe left / right |
| `Q` / `E` (or `←` / `→`) | Rotate left (CCW) / right (CW) |
| `Space` | Enable / Disable toggle |
| `Z` | **LB** — extend intake (start collecting Fuel) |
| `X` | **RB** — retract intake |
| `F` | **A** — fire one Fuel piece at the Hub |
| `G` | **Y** — drive-to-Hub (BLine drive-to-pose) |
| `C` / `V` | **B** / **X** — unbound in the demo (available for your bindings) |

Movement is alliance-aware: "forward" always faces the opponent's wall. The button-key → gamepad-button map lives in `KEY_BUTTON_MAP` in `index.html`; the button → command bindings live in `ExampleRobot.configureBindings()`.

---

## Watching the simulation

WPILib publishes all robot state over **NetworkTables 4 on port 5810**. [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) can connect for a live 3D view.

### Live connection — local machine

1. Start the simulation (see above). Glass opens automatically.
2. In AdvantageScope: **File → Connect to Robot**, enter `localhost` (or `127.0.0.1`) as the host.
3. Drag `RealOutputs/Drive/Pose` onto the 3D field view. Pose, module states, and gyro heading appear in real time.

### Live connection — GitHub Codespaces or cloud environment

Glass cannot render a window in a headless environment. To still observe live state:

1. Run the simulation with the visual-test flag so it auto-enables and drives itself:
   ```bash
   xvfb-run ./gradlew simulateJava -PvisualTest
   ```
2. In VS Code's **Ports** panel, find port **5810** and click the globe icon to get the forwarded URL (it looks like `https://<codespace>-5810.app.github.dev`).
3. In AdvantageScope on your local laptop: **File → Connect to Robot**, paste the forwarded host (without `https://` — just the hostname portion), port `5810`.
4. AdvantageScope receives the live NT4 feed and renders the robot in 3D.

> **Enabling without `-PvisualTest`:** Glass publishes a `DriverStation/Enabled` NT4 entry. If you need interactive control, you can set it to `true` via AdvantageScope's **NT4 → Publish** feature or write a short NT4 client. For most development work, `-PvisualTest` is simpler.

### Log replay (no network required)

AdvantageKit writes a `.wpilog` to the `logs/` directory every run. Share that file; teammates open it with **File → Open Log** and scrub through the full run offline. This is the recommended workflow for reviewing autonomous routines.

---

## GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/clrozeboom/FRC5010Claude)

The `.devcontainer` provides Java 17, Gradle, and all vendordep dependencies pre-downloaded. Port 5810 (NT4) is forwarded automatically so AdvantageScope on your laptop can connect live.

```bash
# Inside the Codespace terminal — headless via xvfb (no display required)
xvfb-run ./gradlew simulateJava -PvisualTest
```

The robot auto-enables and runs the visual-test sequence. Connect AdvantageScope on your local machine using the forwarded port 5810 as described in the [Live connection — cloud environment](#live-connection--github-codespaces-or-cloud-environment) section above.

**Works in Codespaces:**
- Build, test, and headless sim with live AdvantageScope connection
- Log files written to `logs/` and accessible via the VS Code file explorer

**Requires a local machine:**
- Deploying to a RoboRIO (WPILib VS Code extension + USB or network connection)
- Glass GUI (the Driver Station and Field2d panels do not render without a display server)

---

## Claude Code on the web — setup

To use this repo in a [claude.ai/code](https://claude.ai/code) web session:

1. **Install the Claude Code GitHub App** — go to [github.com/apps/claude](https://github.com/apps/claude), click **Install**, and grant it read/write access to this repository. Without write access the agent cannot push commits or create branches.
2. **Allow the required domains** — when creating a new environment, add the domains in the table below to the **network policy** so the Gradle build and all vendordep downloads succeed. See the [environment configuration docs](https://code.claude.com/docs/en/claude-code-on-the-web) for how to configure the allowed-domains list.

### Required allowed domains

| Domain | Purpose |
|--------|---------|
| `services.gradle.org` | Gradle wrapper distribution |
| `plugins.gradle.org` | Gradle Plugin Portal — GradleRIO plugin |
| `frcmaven.wpi.edu` | WPILib Maven — WPILib libraries + AdvantageKit |
| `repo1.maven.org` | Maven Central — JUnit, YAGSL transitive deps |
| `maven.ctr-electronics.com` | CTRE Phoenix 6 |
| `maven.revrobotics.com` | REV Robotics |
| `maven.reduxrobotics.com` | Redux Robotics |
| `docs.home.thethriftybot.com` | ThriftyBot library |
| `maven.photonvision.org` | PhotonVision |
| `yet-another-software-suite.github.io` | YAGSL |
| `3015rangerrobotics.github.io` | PathPlannerLib |
| `jitpack.io` | JitPack — BLine-Lib path-following library (`com.github.edanliahovetsky:BLine-Lib`) |
| `pypi.org` | Python packages for `frc-docs` MCP server (`uvx first-agentic-csa`) |
| `files.pythonhosted.org` | Python package downloads for `frc-docs` MCP server |

---

## How the physics engine is driven

`SwerveFactory.build()` creates a `SwerveDriveSimulation` body and registers it with `SimulatedArena`. Each robot loop, `AkitSwerveDrive.simulationPeriodic()` advances the arena by one 20 ms period (5 sub-ticks × 4 ms each). The updated wheel positions and gyro readings are read back by `ModuleIOSimPhysics` and `GyroIOSimPhysics` during the following `periodic()` call.

**Do not call `drive.simulationPeriodic()` from `Robot.simulationPeriodic()`** — `CommandScheduler.run()` already calls it via the registered subsystem, and a double-call advances physics twice per loop.

See [Architecture](architecture.md) for the per-cycle call order.
