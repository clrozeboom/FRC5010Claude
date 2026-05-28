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

Uses `RealRobotProfile` with your robot's actual `SwerveConstants`. IronMaple runs the physics using your real robot's mass, bumper footprint, and wheel geometry, so the simulation behaves like your actual robot. This is the default when you click **Simulate Robot Code** in VS Code.

```powershell
.\gradlew.bat simulateJava              # Windows — default
```

### 3. Visual auto-test (`-PvisualTest`)

Same as scenario 2, but `getAutonomousCommand()` returns the `SwerveVisualTest` sequence: the robot drives forward, strafes, rotates, verifies alliance-direction, and approaches the field boundary. Useful for a quick sanity check after changing constants.

```powershell
.\gradlew.bat simulateJava -PvisualTest
```

Click **Enable** in the **Glass → Driver Station** panel (or enable Autonomous mode) to start the sequence.

---

## Watching the simulation

WPILib publishes all robot state over **NetworkTables 4 on port 5810**. [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) can connect for a live 3D view.

### Live connection (same LAN or Codespace)

1. Start the simulation (see above).
2. In AdvantageScope: **File → Connect to Robot**, enter your machine's IP and port `5810`.
   - In a Codespace: copy the forwarded URL for port 5810 from VS Code's **Ports** panel.
3. Pose, module states, and gyro heading appear in real time.

### Log replay (no network required)

AdvantageKit writes a `.wpilog` to the `logs/` directory every run. Share that file; teammates open it with **File → Open Log** and scrub through the full run offline. This is the recommended workflow for reviewing autonomous routines.

---

## GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/clrozeboom/FRC5010Claude)

The `.devcontainer` provides Java 17, Gradle, and all vendordep dependencies pre-downloaded. Port 5810 (NT4) is forwarded automatically.

```bash
# Inside the Codespace terminal — headless via xvfb
xvfb-run ./gradlew simulateJava
```

**Works in Codespaces:**
- Build, test, and headless sim
- NT4 live connection to AdvantageScope via the forwarded port

**Requires a local machine:**
- Deploying to a RoboRIO (WPILib VS Code extension + USB or network connection)

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
| `pypi.org` | Python packages for `frc-docs` MCP server (`uvx first-agentic-csa`) |
| `files.pythonhosted.org` | Python package downloads for `frc-docs` MCP server |

---

## How the physics engine is driven

`SwerveFactory.build()` creates a `SwerveDriveSimulation` body and registers it with `SimulatedArena`. Each robot loop, `AkitSwerveDrive.simulationPeriodic()` advances the arena by one 20 ms period (5 sub-ticks × 4 ms each). The updated wheel positions and gyro readings are read back by `ModuleIOSimPhysics` and `GyroIOSimPhysics` during the following `periodic()` call.

**Do not call `drive.simulationPeriodic()` from `Robot.simulationPeriodic()`** — `CommandScheduler.run()` already calls it via the registered subsystem, and a double-call advances physics twice per loop.

See [Architecture](architecture.md) for the per-cycle call order.
